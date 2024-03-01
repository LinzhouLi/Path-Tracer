
#include <pt/scene.h>
#include <pt/vector.h>
#include <pt/color.h>
#include <pt/mesh.h>
#include <pt/bitmap.h>
#include <pt/material.h>
#include <pt/camera.h>
#include <pt/accel.h>
#include <pt/integrator.h>
#include <pt/shape.h>
#include <pt/sampler.h>
#include <pt/light.h>

#include <pugixml.hpp>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace pt {

float toFloat(const std::string& str) {
	char* end_ptr = nullptr;
	float result = (float)strtof(str.c_str(), &end_ptr);
	if (*end_ptr != '\0')
		throw PathTracerException("Could not parse floating point value \"%s\"", str);
	return result;
}

void Scene::loadOBJ(const std::string& filename) {
	cout << "Reading a OBJ file from \"" << filename << "\" ..." << endl;

	tinyobj::ObjReaderConfig reader_config;
	reader_config.vertex_color = false; // no vertex color
	tinyobj::ObjReader reader;

	bool sucess = reader.ParseFromFile(filename, reader_config);

	if (!sucess)
		throw PathTracerException("Fail to read!");

	const tinyobj::attrib_t& attrib = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	const std::vector<tinyobj::material_t>& materials = reader.GetMaterials();

	// vertex data
	std::vector<Vector3f> vertices(attrib.vertices.size() / 3);
	std::vector<Vector3f> normals(attrib.normals.size() / 3);
	std::vector<Vector2f> uvs(attrib.texcoords.size() / 2);

	for (size_t i = 0; i < vertices.size(); i++)
		vertices[i] = Vector3f(attrib.vertices[3 * i], attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2]);
	for (size_t i = 0; i < normals.size(); i++)
		normals[i] = Vector3f(attrib.normals[3 * i], attrib.normals[3 * i + 1], attrib.normals[3 * i + 2]);
	for (size_t i = 0; i < uvs.size(); i++)
		uvs[i] = Vector2f(attrib.texcoords[2 * i], attrib.texcoords[2 * i + 1]);

	// face data
	std::vector<uint32_t> vertex_ids, normal_ids, uv_ids;
	std::vector<uint32_t> mtl_ids;

	for (size_t s = 0; s < shapes.size(); s++) {
		tinyobj::shape_t shape = shapes[s];

		// check triangle
		std::vector<uint32_t>& faceVertNums = shape.mesh.num_face_vertices;
		auto it = std::find_if(faceVertNums.begin(), faceVertNums.end(), [](uint32_t i) { return i != 3; });
		if (it != faceVertNums.end())
			throw PathTracerException("Contains non-triangle face! Only support OBJ file with triangle faces.");

		size_t numFaces = faceVertNums.size();
		for (size_t f = 0; f < numFaces; f++) {
			mtl_ids.push_back(shape.mesh.material_ids[f] + this->m_materials.size()); // local mtl id -> global mtl id
			for (size_t v : {0, 1, 2}) {
				tinyobj::index_t idx = shapes[s].mesh.indices[3 * f + v];
				vertex_ids.push_back(idx.vertex_index);
				normal_ids.push_back(idx.normal_index);
				uv_ids.push_back(idx.texcoord_index);
			}
		}
	}

	TriangleMesh* mesh = new TriangleMesh(
		filename, vertices, normals, uvs, 
		vertex_ids, normal_ids, uv_ids, mtl_ids
	);
	this->m_meshes.push_back(mesh);

	cout << "Load a mesh with " << mesh->getVertexCount() << " vertices and " << mesh->getTriangleCount() << " triangles!" << endl;

	// Loop over materials
	for (size_t m = 0; m < materials.size(); m++) {
		tinyobj::material_t material = materials[m];

		Material* material_ = new Material(
			material.name,
			Vector3f(material.diffuse[0], material.diffuse[1], material.diffuse[2]),
			Vector3f(material.specular[0], material.specular[1], material.specular[2]),
			Vector3f(material.transmittance[0], material.transmittance[1], material.transmittance[2]),
			material.shininess, material.ior
		);

		// load texture
		if (!material.diffuse_texname.empty()) { 
			// find texture file under the same folder with OBJ file.
			size_t pos = filename.find_last_of("/\\");
			std::string baseDir = pos == std::string::npos ? "" : filename.substr(0, pos);
			if (!baseDir.empty()) {
				if (baseDir[baseDir.length() - 1] != DIR_SEP) baseDir += DIR_SEP;
			}

			// load bitmap data
			Bitmap* bitmap = new Bitmap;
			bitmap->load(baseDir + material.diffuse_texname);
			material_->setTexture(bitmap);
		}

		this->m_materials.push_back(material_);
	}

	cout << "Load " << this->m_materials.size() << " materials!" << endl;
}

void Scene::loadXML(const std::string& filename) {
	cout << "Reading a XML file from \"" << filename << "\" ..." << endl;

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(filename.c_str());

	if (!result)
		throw PathTracerException(result.description());

	pugi::xml_node camera_node = doc.child("camera");

	if (std::string(camera_node.attribute("type").value()) != "perspective")
		throw PathTracerException("Only support perspective camera!");

	pugi::xml_node camera_eye_node = camera_node.child("eye");
	pugi::xml_node camera_lookat_node = camera_node.child("lookat");
	pugi::xml_node camera_up_node = camera_node.child("up");

	Vector3f eye(
		camera_eye_node.attribute("x").as_float(),
		camera_eye_node.attribute("y").as_float(),
		camera_eye_node.attribute("z").as_float()
	);

	Vector3f lookat(
		camera_lookat_node.attribute("x").as_float(),
		camera_lookat_node.attribute("y").as_float(),
		camera_lookat_node.attribute("z").as_float()
	);

	Vector3f up(
		camera_up_node.attribute("x").as_float(),
		camera_up_node.attribute("y").as_float(),
		camera_up_node.attribute("z").as_float()
	);

	this->m_camera = new Camera(
		camera_node.attribute("width").as_uint(),
		camera_node.attribute("height").as_uint(),
		camera_node.attribute("fovy").as_float(),
		eye, lookat, up
	);

	cout << "Load a camera!" << endl;

	auto light_nodes = doc.children("light");
	for (pugi::xml_node light_node : light_nodes) {
		std::string light_name = light_node.attribute("mtlname").value();
		const char* radiance_str = light_node.attribute("radiance").as_string();
		float r = toFloat(std::strtok(const_cast<char*>(radiance_str), ","));
		float g = toFloat(std::strtok(nullptr, ","));
		float b = toFloat(std::strtok(nullptr, ","));
		Vector3f radiance(r, g, b);

		m_light_infos.push_back(LightInfo(light_name, radiance));
	}

	cout << "Load " << this->m_light_infos.size() << " area lights!" << endl;
}

TriangleMesh* Scene::getMesh(const std::string& mesh_name) {
	for (TriangleMesh* mesh : m_meshes)
		if (mesh->getName() == mesh_name) return mesh;
	return nullptr;
}

Material* Scene::getMaterial(const std::string& material_name) {
	for (Material* material : m_materials)
		if (material->getName() == material_name) return material;
	return nullptr;
}

Material* Scene::getMaterial(const Intersection& its) {
	return m_materials[its.getShape()->getMaterialId()];
}

TriangleMesh* Scene::getMesh(const uint32_t mesh_id) {
	if (mesh_id >= 0 && mesh_id < m_meshes.size())
		return m_meshes[mesh_id];
	else
		throw PathTracerException("Invalid mesh index!");
}

Material* Scene::getMaterial(const uint32_t material_id) {
	if (material_id >= 0 && material_id < m_materials.size())
		return m_materials[material_id];
	else
		throw PathTracerException("Invalid material index!");
}

bool Scene::rayIntersect(const Ray& ray, Intersection& its) const {
	return m_accel->rayIntersect(ray, its);
}

bool Scene::unocculded(Vector3f p0, Vector3f p1, const Vector3f& n0, const Vector3f& n1) const {
	p0 += n0 * Epsilon;
	p1 += n1 * Epsilon;
	Vector3f d = p1 - p0;
	float dist = d.norm();
	Ray ray(p0, d / dist, 0, dist * (1 - Epsilon));
	return !m_accel->rayIntersect(ray);
}

void Scene::preprocess() {
	// create primitives & lights
	createPrimitives();
	createAreaLights();

	// build accelration
	m_accel = new BVHTree(&m_shapes);
	m_accel->build();

	// build Integrator
	m_integrator = new PathIntegrator();
	m_integrator->preprocess(this);

	// create sampler
	//m_sampler = new SobolSampler(m_spp, m_camera->getScreenSize());
	m_sampler = new IndependentSampler(m_spp);
}

void Scene::createPrimitives() {
	uint32_t total_triangles = 0;
	for (uint32_t i = 0; i < m_meshes.size(); i++) {
		total_triangles += m_meshes[i]->getTriangleCount();
		for (uint32_t j = 0; j < m_meshes[i]->getTriangleCount(); j++)
			m_shapes.push_back(new Triangle(j, m_meshes[i]));
	}
	cout << "Create " << total_triangles << " primitives!" << endl;
}

void Scene::createAreaLights() {
	for (LightInfo& info : m_light_infos) {
		for (Triangle* shape : m_shapes) {
			if (getMaterial(shape->getMaterialId())->getName() == info.mtl_name) {
				AreaLight* light = new AreaLight(shape, info.radiance);
				m_lights.push_back(light);
				shape->setLight(light);
			}
		}
	}
	cout << m_lights.size() << endl;
}

}