#pragma once

#include <pt/common.h>
#include <pt/aabb.h>

namespace pt {

// Brute force method
class Accel {
public:
	Accel(std::vector<Triangle*>* primitives) : m_primitives(primitives) { }

	virtual void build();

	virtual bool rayIntersect(const Ray& ray, Intersaction& its);

protected:
	std::vector<Triangle*>* m_primitives;
};

// BVH tree accelration
class BVHTree : public Accel {
public:
	BVHTree(std::vector<Triangle*>* primitives) : Accel(primitives) { }

	void build();

	bool rayIntersect(const Ray& ray, Intersaction& its);

private:
	 std::vector<AABB> aabbs;
};

}