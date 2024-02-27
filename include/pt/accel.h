#pragma once

#include <pt/common.h>

namespace pt {

// Brute force method
class Accel {
public:
	void addMesh(Mesh* mesh);

	virtual void build();

	virtual bool rayIntersect(const Ray& ray, Intersaction& its);

protected:
	Mesh* m_mesh = nullptr;
};

// BVH tree accelration
class BVHTree : public Accel {
public:
	void build();

	bool rayIntersect(const Ray& ray, Intersaction& its);

private:
	std::vector<AABB> aabbs;
};

}