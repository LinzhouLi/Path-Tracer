#include <pt/shape.h>
#include <pt/mesh.h>
#include <pt/aabb.h>
#include <pt/ray.h>
#include <pt/light.h>
#include <pt/material.h>
#include <pt/ray.h>

namespace pt {

void Triangle::getVertex(Vector3f& v0, Vector3f& v1, Vector3f& v2) const {
	TriangleMesh* mesh = getMesh();
	mesh->getVertex(m_triangle_id, v0, v1, v2);
}

bool Triangle::getNormal(Vector3f& n0, Vector3f& n1, Vector3f& n2) const {
	TriangleMesh* mesh = getMesh();
	return mesh->getNormal(m_triangle_id, n0, n1, n2);
}

bool Triangle::getUV(Vector2f& uv0, Vector2f& uv1, Vector2f& uv2) const {
	TriangleMesh* mesh = getMesh();
	return mesh->getUV(m_triangle_id, uv0, uv1, uv2);
}

//uint32_t Triangle::getMaterialId() const {
//	TriangleMesh* mesh = getMesh();
//	return mesh->getMaterialId(m_triangle_id);
//}


float Triangle::surfaceArea() const {
	Vector3f v0, v1, v2;
	getVertex(v0, v1, v2);
	return 0.5f * Vector3f((v1 - v0).cross(v2 - v0)).norm();
}

AABB Triangle::getAABB() const{
	Vector3f v0, v1, v2;
	getVertex(v0, v1, v2);
	return AABB(v0, v1, v2);
}

Vector3f Triangle::getCenter() const {
	Vector3f v0, v1, v2;
	getVertex(v0, v1, v2);
	return (v0 + v1 + v2) / 3.0f;
}

bool Triangle::intersect(const Ray& ray, Vector3f& bary, float& t) const {
	Vector3f p0, p1, p2;
	getVertex(p0, p1, p2);

	/* Find vectors for two edges sharing v[0] */
	Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

	/* Begin calculating determinant - also used to calculate U parameter */
	Vector3f pvec = ray.dir.cross(edge2);

	/* If determinant is near zero, ray lies in plane of triangle */
	float det = edge1.dot(pvec);
	if (det > -1e-5f && det < 1e-5f)
		return false;

	float inv_det = 1.0f / det;

	/* Calculate distance from v[0] to ray origin */
	Vector3f tvec = ray.org - p0;

	/* Calculate U parameter and test bounds */
	float u = tvec.dot(pvec) * inv_det;
	if (u < 0.0 || u > 1.0)
		return false;

	/* Prepare to test V parameter */
	Vector3f qvec = tvec.cross(edge1);

	/* Calculate V parameter and test bounds */
	float v = ray.dir.dot(qvec) * inv_det;
	if (v < 0.0 || u + v > 1.0)
		return false;

	/* Ray intersects triangle -> compute t */
	t = edge2.dot(qvec) * inv_det;

	bary << 1 - u - v, u, v;

	return t >= ray.min_dis && t <= ray.max_dis;
}

/**
* Shirley, P.et al. (2019).Sampling Transformations Zoo.In: Haines, E., Akenine - Möller, T.
* (eds)Ray Tracing Gems.Apress, Berkeley, CA.https ://doi.org/10.1007/978-1-4842-4427-2_16
*/
TriangleSample Triangle::sample(const Vector2f& u) const {
	float su0 = std::sqrt(u.x());
	float bary0 = 1 - su0, bary1 = u.y() * su0;
	float bary2 = 1 - bary0 - bary1;

	Vector3f v0, v1, v2, n0, n1, n2;
	getVertex(v0, v1, v2);
	Vector3f p = v0 * bary0 + v1 * bary1 + v2 * bary2;

	Vector3f n;
	if (getNormal(n0, n1, n2))
		n = n0 * bary0 + n1 * bary1 + n2 * bary2;
	else
		n = ((v0 - v2).cross(v1 - v2));
	n.normalize();

	float pdf = 1.0 / surfaceArea();
	return TriangleSample { p, n, pdf };
}

std::string Triangle::toString() const {
	return tfm::format(
		"Triangle[\n"
		"  triangle_id = %i,\n"
		"  area = %f,\n"
		"  aabb = %s\n"
		"]",
		m_triangle_id,
		surfaceArea(),
		getAABB().toString()
	);
}


Intersection& Intersection::operator= (const Intersection& cls) {
	p = cls.p;
	n = cls.n;
	ng = cls.ng;
	uv = cls.uv;
	ts = cls.ts;
	m_shape = cls.m_shape;
	m_bary = cls.m_bary;
	return *this;
}

void Intersection::setInfo(const Triangle* shape, const Vector3f& bary) {
	m_shape = shape;
	m_bary = bary;
}

void Intersection::complete() {
	// intersact position
	Vector3f v0, v1, v2;
	m_shape->getVertex(v0, v1, v2);
	p = v0 * m_bary.x() + v1 * m_bary.y() + v2 * m_bary.z();

	// normal
	Vector3f n0, n1, n2;
	ng = n = ((v0 - v2).cross(v1 - v2));
	if (m_shape->getNormal(n0, n1, n2))
		n = n0 * m_bary.x() + n1 * m_bary.y() + n2 * m_bary.z();
	n.normalize(); // shading normal
	ng.normalize(); // geometric normal

	// tangent
	ts = TangentSpace(n);

	// uv
	Vector2f uv0(0.0f, 0.0f), uv1(1.0f, 0.0f), uv2(1.0f, 1.0f);
	m_shape->getUV(uv0, uv1, uv2); // rewrite if shape has uv
	uv = uv0 * m_bary.x() + uv1 * m_bary.y() + uv2 * m_bary.z();
}
	
Vector3f Intersection::Le(const Vector3f& w) const {
	AreaLight* light = m_shape->getLight();
	if (light) return light->L(n, w);
	else return Vector3f(0.0f);
}

Vector3f Intersection::BRDF(const Vector3f& wo, const Vector3f& wi) const {
	return m_shape->getMaterial()->BRDF(wo, wi, *this);
}

BRDFSample Intersection::sampleBRDF(const Vector3f& wo, float uc, const Vector2f& u) const {
	return m_shape->getMaterial()->sampleBRDF(wo, uc, u, *this);
}

float Intersection::pdfBRDF(const Vector3f& wo, const Vector3f& wi) const {
	return m_shape->getMaterial()->pdf(wo, wi, *this);
}

Ray Intersection::genRay(const Vector3f& w) const {
	Vector3f p_ = p + n * Epsilon;
	return Ray(p_, w, 0);
}

}