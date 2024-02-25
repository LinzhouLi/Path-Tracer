
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

	// load attributes
	size_t numVertices = attrib.vertices.size() / 3;
	vertices.resize(numVertices);
	for (size_t i = 0; i < numVertices; i++) {
		vertices[i] = Point3f(
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

	}
}

}