#pragma once

#include <pt/common.h>

namespace pt {

class Accel {
public:
	void addMesh(Mesh* mesh);

	void build();

	bool rayIntersect(const Ray& ray);

private:
	Mesh* m_mesh = nullptr;
};

}