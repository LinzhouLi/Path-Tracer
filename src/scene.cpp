
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

#include <pugixml.hpp>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace pt {

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
			for (size_t v = 0; v < 3; v++) {
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

	auto light_nodes = doc.children("light");
	for (pugi::xml_node light_node : light_nodes) {
		//cout << light_node.attribute("mtlname").value() << endl;
		const char* radiance_str = light_node.attribute("radiance").as_string();
		float r = toFloat(std::strtok(const_cast<char*>(radiance_str), ","));
		float g = toFloat(std::strtok(nullptr, ","));
		float b = toFloat(std::strtok(nullptr, ","));
		Vector3f radiance(r, g, b);
		//cout << radiance.toString() << endl;
		// TODO: add light
	}
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

bool Scene::rayIntersect(const Ray& ray, Intersaction& its) const {
	return m_accel->rayIntersect(ray, its);
}

void Scene::preprocess() {
	// create primitives
	Triangle::setMeshGroup(&m_meshes);
	createPrimitives();

	// build accelration
	m_accel = new Accel(&m_primitives);
	m_accel->build();

	// build Integrator
	//m_integrator = new BaseColorIntegrator();
	m_integrator = new GeometryIntegrator();
	m_integrator->preprocess(this);
}

void Scene::createPrimitives() {
	uint32_t total_triangles = 0;
	for (uint32_t i = 0; i < m_meshes.size(); i++) {
		total_triangles += m_meshes[i]->getTriangleCount();
		for (uint32_t j = 0; j < m_meshes[i]->getTriangleCount(); j++)
			m_primitives.push_back(new Triangle(i, j));
	}
	cout << "Create " << total_triangles << " primitives!" << endl;
}

}