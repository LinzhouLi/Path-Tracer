#include <pt/aabb.h>
#include <pt/ray.h>

namespace pt {

bool AABB::intersect(const Ray& ray) const {
	float tmin = (m_min.x() - ray.org.x()) / ray.dir.x();
	float tmax = (m_max.x() - ray.org.x()) / ray.dir.x();

	if (tmin > tmax) std::swap(tmin, tmax);

	float tymin = (m_min.y() - ray.org.y()) / ray.dir.y();
	float tymax = (m_max.y() - ray.org.y()) / ray.dir.y();

	if (tymin > tymax) std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (m_min.z() - ray.org.z()) / ray.dir.z();
	float tzmax = (m_max.z() - ray.org.z()) / ray.dir.z();

	if (tzmin > tzmax) std::swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	return (tmin < ray.max_dis) && (tmax > ray.min_dis);
}

}