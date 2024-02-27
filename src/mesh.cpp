#include <pt/mesh.h>
#include <pt/ray.h>

namespace pt{

float triangleSurfaceArea(const Mesh::TriVertex& triangle) {
	const Vector3f& p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];

	return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool rayTriangleIntersect(const Mesh::TriVertex& triangle, const Ray& ray, float& u, float& v, float& t) {
	const Vector3f& p0 = triangle[0], p1 = triangle[1], p2 = triangle[2];

	/* Find vectors for two edges sharing v[0] */
	Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

	/* Begin calculating determinant - also used to calculate U parameter */
	Vector3f pvec = ray.dir.cross(edge2);

	/* If determinant is near zero, ray lies in plane of triangle */
	float det = edge1.dot(pvec);
	if (det > -1e-8f && det < 1e-8f)
		return false;

	float inv_det = 1.0f / det;

	/* Calculate distance from v[0] to ray origin */
	Vector3f tvec = ray.org - p0;

	/* Calculate U parameter and test bounds */
	u = tvec.dot(pvec) * inv_det;
	if (u < 0.0 || u > 1.0)
		return false;

	/* Prepare to test V parameter */
	Vector3f qvec = tvec.cross(edge1);

	/* Calculate V parameter and test bounds */
	v = ray.dir.dot(qvec) * inv_det;
	if (v < 0.0 || u + v > 1.0)
		return false;

	/* Ray intersects triangle -> compute t */
	t = edge2.dot(qvec) * inv_det;

	return t >= ray.min_dis && t <= ray.max_dis;
}

}