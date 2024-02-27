#pragma once

#include <pt/common.h>

namespace pt {

class Accel {
public:
	void addMesh(Mesh* mesh);

	virtual void build();

	virtual bool rayIntersect(const Ray& ray, Intersaction& its);

private:
	Mesh* m_mesh = nullptr;
};

class BVHTree : public Accel {
public:
	void build();

	bool rayIntersect(const Ray& ray, Intersaction& its);
};

}