
#include <pt/scene.h>
#include <pt/vector.h>
#include <pt/color.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace pt {

void Scene::loadOBJ(const std::string& filename) {
	tinyobj::ObjReaderConfig reader_config;
	tinyobj::ObjReader reader;

	bool sucess = reader.ParseFromFile(filename, reader_config);

	if (!sucess)
		throw PathTracerException(("Fail to load OBJ file: \"" + filename + "\"!").c_str());

	cout << "Reading a OBJ file from \"" << filename << "\"" << endl;

	const tinyobj::attrib_t& attrib = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	const std::vector<tinyobj::material_t>& materials = reader.GetMaterials();

	// load global attributes
	size_t numVertices = attrib.vertices.size() / 3;
	this->vertices.resize(numVertices);
	for (size_t i = 0; i < numVertices; i++) {
		this->vertices[i] = Point3f(
			attrib.vertices[3 * i + 0],
			attrib.vertices[3 * i + 1],
			attrib.vertices[3 * i + 2]
		);
	}

	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		tinyobj::shape_t shape = shapes[s];
		std::string meshName = shape.name;

		// check triangle
		std::vector<unsigned int>& faceVertNums = shape.mesh.num_face_vertices;
		auto it = std::find_if_not(faceVertNums.begin(), faceVertNums.end(), 3);
		if (it != faceVertNums.end())
			throw PathTracerException("Contains non-triangle face! Only support OBJ file with triangle faces.");

		// mesh attributes
		size_t numFaces = faceVertNums.size();
		std::vector<Point3i> faces(numFaces);
		std::vector<std::array< Normal3f, 3 >> normals(numFaces);
		std::vector<std::array< Point2f, 3 >> uvs(numFaces);

		for (size_t f = 0; f < numFaces; f++) {
			faces[f] = Point3i(
				shape.mesh.indices[3 * f + 0].vertex_index,
				shape.mesh.indices[3 * f + 1].vertex_index,
				shape.mesh.indices[3 * f + 2].vertex_index
			);
			for (size_t v = 0; v < 3; v++) {
				tinyobj::index_t idx = shapes[s].mesh.indices[3 * f + v];
				if (idx.normal_index >= 0) {
					normals[f][v] = Normal3f(
						attrib.normals[3 * idx.normal_index + 0],
						attrib.normals[3 * idx.normal_index + 1],
						attrib.normals[3 * idx.normal_index + 2]
					);
				}
				if (idx.normal_index >= 0) {
					uvs[f][v] = Point2f(
						attrib.texcoords[2 * idx.normal_index + 0],
						attrib.texcoords[2 * idx.normal_index + 1]
					);
				}
			}
		}


	}
}

}