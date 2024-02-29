#pragma once

#include <pt/common.h>
#include <pt/vector.h>

namespace pt {

class Triangle {
public:
	static void setMeshGroup(std::vector<TriangleMesh*>* meshes) { m_meshes = meshes; }

	Triangle(uint32_t mesh_id, uint32_t triangle_id) : m_mesh_id(mesh_id), m_triangle_id(triangle_id) { }

	// Calculate surface area
	float surfaceArea() const;

	// Get AABB bonding box
	AABB getAABB() const;

	// Ray intersect with triangle
	bool intersect(const Ray& ray, Vector3f& bary, float& t) const;

	// Get geometry infomation
	void getVertex(Vector3f& v0, Vector3f& v1, Vector3f& v2) const;
	bool getNormal(Vector3f& n0, Vector3f& n1, Vector3f& n2) const;
	bool getUV(Vector2f& uv0, Vector2f& uv1, Vector2f& uv2) const;

	// Get material ID
	uint32_t getMaterialId() const;

private:
	TriangleMesh* getMesh() const { return (*Triangle::m_meshes)[m_mesh_id]; }

	static std::vector<TriangleMesh*>* m_meshes;

	uint32_t m_mesh_id, m_triangle_id;
};


class Intersection {
public:
	void setInfo(Triangle* shape, const Vector3f& bary);

	Triangle* getShape() { return m_shape; }

	void complete();

	Vector3f pos;
	Vector3f geo_n; // geometry normal
	Vector3f sha_n; // shading normal
	Vector2f uv;

private:
	Triangle* m_shape = nullptr;
	Vector3f m_bary;
};

}