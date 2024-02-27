#include <pt/mesh.h>
#include <pt/ray.h>

namespace pt{

float triangleSurfaceArea(const Mesh::TriVertex& triangle) {
	const Vector3f& p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];

	return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool rayTriangleIntersect(const Mesh::TriVertex& triangle, const Ray& ray, Vector3f& bary, float& t) {
	const Vector3f& p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];

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


void Intersaction::setInfo(Mesh* mesh, uint32_t triangle_idx, const Vector3f& bary) {
	m_mesh = mesh;
	m_triangle_idx = triangle_idx;
	m_bary = bary;
}

void Intersaction::complete() {
	Mesh::TriVertex tri_pos = m_mesh->getTriangle(m_triangle_idx);
	Mesh::TriNormal tri_normal = m_mesh->getNormal(m_triangle_idx);
	Mesh::TriUV tri_uv = m_mesh->getUV(m_triangle_idx);

	pos = tri_pos[0] * m_bary.x() + tri_pos[1] * m_bary.y() + tri_pos[2] * m_bary.z();
	normal = tri_normal[0] * m_bary.x() + tri_normal[1] * m_bary.y() + tri_normal[2] * m_bary.z();
	normal.normalize();
	uv = tri_uv[0] * m_bary.x() + tri_uv[1] * m_bary.y() + tri_uv[2] * m_bary.z();
	mat_id = m_mesh->getMaterialId(m_triangle_idx);
}

}