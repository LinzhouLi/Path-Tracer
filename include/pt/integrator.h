#pragma once

#include <pt/common.h>

namespace pt {

class Integrator {
public:
	virtual void preprocess(Scene* scene) { }

	virtual Color3f Li(Scene* scene, const Ray& ray) const = 0;
};

class GeometryIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build GeometryIntegrator!" << endl;
	}

	Color3f Li(Scene* scene, const Ray& ray) const;
};

class BaseColorIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build BaseColorIntegrator!" << endl;
	}

	Color3f Li(Scene* scene, const Ray& ray) const;
};

}