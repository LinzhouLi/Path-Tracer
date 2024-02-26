
#include <pt/scene.h>
#include <pt/vector.h>
#include <pt/color.h>
#include <pt/mesh.h>
#include <pt/bitmap.h>
#include <pt/material.h>
#include <pt/camera.h>

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

	// Loop over shapes(meshes)
	for (size_t s = 0; s < shapes.size(); s++) {
		tinyobj::shape_t shape = shapes[s];
		std::string meshName = shape.name;

		// check triangle
		std::vector<uint32_t>& faceVertNums = shape.mesh.num_face_vertices;
		auto it = std::find_if(faceVertNums.begin(), faceVertNums.end(), [](uint32_t i) { return i != 3; });
		if (it != faceVertNums.end())
			throw PathTracerException("Contains non-triangle face! Only support OBJ file with triangle faces.");

		// mesh attributes
		size_t numFaces = faceVertNums.size();
		std::vector<Mesh::TriVertex> vertices(numFaces);
		std::vector<Mesh::TriNormal> normals(numFaces);
		std::vector<Mesh::TriUV> uvs(numFaces);
		std::vector<uint32_t> mat_ids(numFaces);

		// per face data
		for (size_t f = 0; f < numFaces; f++) {
			mat_ids[f] = shape.mesh.material_ids[f];
			for (size_t v = 0; v < 3; v++) {
				tinyobj::index_t idx = shapes[s].mesh.indices[3 * f + v];
				vertices[f][v] = Vector3f( // load vertex
					attrib.vertices[3 * idx.vertex_index + 0],
					attrib.vertices[3 * idx.vertex_index + 1],
					attrib.vertices[3 * idx.vertex_index + 2]
				);
				if (idx.normal_index >= 0) { // load normal
					normals[f][v] = Vector3f(
						attrib.normals[3 * idx.normal_index + 0],
						attrib.normals[3 * idx.normal_index + 1],
						attrib.normals[3 * idx.normal_index + 2]
					);
				}
				else throw PathTracerException("No normal data!");
				if (idx.texcoord_index >= 0) { // load uv
					uvs[f][v] = Vector2f(
						attrib.texcoords[2 * idx.texcoord_index + 0],
						attrib.texcoords[2 * idx.texcoord_index + 1]
					);
				}
				else throw PathTracerException("No uv data!");
			}
		}

		Mesh* mesh = new Mesh(meshName, vertices, normals, uvs, mat_ids);
		this->m_meshes.push_back(mesh);
	}

	cout << "Load " << this->m_meshes.size() << " mesh!" << endl;

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

	cout << "Load " << this->m_materials.size() << " material!" << endl;
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

Mesh* Scene::getMesh(const std::string& mesh_name) {
	for (Mesh* mesh : m_meshes)
		if (mesh->getName() == mesh_name) return mesh;
	return nullptr;
}

Material* Scene::getMaterial(const std::string& material_name) {
	for (Material* material : m_materials)
		if (material->getName() == material_name) return material;
	return nullptr;
}

Mesh* Scene::getMesh(const uint32_t mesh_id) {
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

}