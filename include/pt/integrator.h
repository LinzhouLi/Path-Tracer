#pragma once

#include <pt/common.h>

namespace pt {

class Integrator {
public:
	virtual void preprocess(const Scene* scene) { }

	virtual Color3f Li(const Scene* scene, const Ray& ray) const = 0;
};

class GeometryIntegrator : public Integrator {
public:
	void preprocess(const Scene* scene);

	Color3f Li(const Scene* scene, const Ray& ray) const;
};

}