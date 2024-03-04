#pragma once

#include <pt/vector.h>
#include <pt/tangent.h>

namespace pt {

struct BRDFSample;

struct TriangleSample {
	TriangleSample(const Vector3f& p_, const Vector3f& n_, float pdf_) :
		p(p_), n(n_), pdf(pdf_) { }

	Vector3f p;
	Vector3f n;
	float pdf; // area measure
};

class Triangle {
public:
	Triangle(uint32_t triangle_id, TriangleMesh* mesh, Material* material, AreaLight* light = nullptr) : 
		m_triangle_id(triangle_id), m_mesh(mesh), m_material(material), m_light(light) { }

	// Calculate surface area
	float surfaceArea() const;

	// Get AABB bonding box
	AABB getAABB() const;

	// Ray intersect with triangle
	bool intersect(const Ray& ray, Vector3f& bary, float& t) const;

	// Sample a point on triangle
	TriangleSample sample(const Vector2f& u) const;

	// PDF
	float pdf() const { return 1.0 / surfaceArea(); }

	// Get geometry infomation
	void getVertex(Vector3f& v0, Vector3f& v1, Vector3f& v2) const;
	bool getNormal(Vector3f& n0, Vector3f& n1, Vector3f& n2) const;
	bool getUV(Vector2f& uv0, Vector2f& uv1, Vector2f& uv2) const;

	// Get material
	const Material* getMaterial() const { return m_material;  };

	// Get mesh
	TriangleMesh* getMesh() const { return m_mesh; }

	// Get area light
	AreaLight* getLight() const { return m_light; }

	// Set area light
	void setLight(AreaLight* light) { m_light = light; }

private:
	uint32_t m_triangle_id;
	TriangleMesh* m_mesh = nullptr;
	const Material* m_material = nullptr;
	AreaLight* m_light = nullptr;

};


class Intersection {
public:
	Intersection() : ts(Vector3f(0.0f, 0.0f, 1.0f)) { }

	void setInfo(const Triangle* shape, const Vector3f& bary);

	const Triangle* getShape() const { return m_shape; }

	const Material* getMaterial() const { return m_shape->getMaterial(); }

	const AreaLight* getLight() const { return m_shape->getLight(); }

	void complete();

	Vector3f Le(const Vector3f& w) const;

	Vector3f BRDF(const Vector3f& wo, const Vector3f& wi) const;

	BRDFSample sampleBRDF(const Vector3f& wo, float uc, const Vector2f& u) const;

	float pdfBRDF(const Vector3f& wo, const Vector3f& wi) const;

	Ray genRay(const Vector3f& w) const;

	Vector3f p;
	Vector3f n; // shading normal
	Vector2f uv;
	TangentSpace ts;

private:
	const Triangle* m_shape = nullptr;
	Vector3f m_bary;
};

}