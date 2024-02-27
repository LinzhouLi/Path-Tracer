#pragma once

#include <pt/common.h>

namespace pt {

class Accel {
public:
	void addMesh(Mesh* mesh);

	void build();

	bool rayIntersect(const Ray& ray, Intersaction& its);

private:
	Mesh* m_mesh = nullptr;
};

}