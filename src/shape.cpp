#include <pt/shape.h>
#include <pt/vector.h>
#include <pt/mesh.h>
#include <pt/aabb.h>
#include <pt/ray.h>

namespace pt {

std::vector<TriangleMesh*>* Triangle::m_meshes = nullptr;

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

uint32_t Triangle::getMaterialId() const {
	TriangleMesh* mesh = getMesh();
	return mesh->getMaterialId(m_triangle_id);
}


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


void Intersaction::setInfo(Triangle* shape, const Vector3f& bary) {
	m_shape = shape;
	m_bary = bary;
}

void Intersaction::complete() {
	// intersact position
	Vector3f v0, v1, v2;
	m_shape->getVertex(v0, v1, v2);
	pos = v0 * m_bary.x() + v1 * m_bary.y() + v2 * m_bary.z();

	// normal
	sha_n = geo_n = ((v0 - v2).cross(v1 - v2)).normalized();
	Vector3f n0, n1, n2;
	if (m_shape->getNormal(n0, n1, n2)) {
		sha_n = n0 * m_bary.x() + n1 * m_bary.y() + n2 * m_bary.z();
		if (sha_n.norm() > 0) sha_n.normalize(); 
		else sha_n = geo_n; // invalid when normal length == 0 (this should be checked when mesh loading)
	}

	// uv
	Vector2f uv0(0.0f, 0.0f), uv1(1.0f, 0.0f), uv2(1.0f, 1.0f);
	m_shape->getUV(uv0, uv1, uv2); // rewrite if shape has uv
	uv = uv0 * m_bary.x() + uv1 * m_bary.y() + uv2 * m_bary.z();
}
	
}