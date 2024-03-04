#pragma once

#include <pt/common.h>

namespace pt {

struct LightSample;

class Integrator {
public:
	virtual void preprocess(Scene* scene) { }

	virtual Color3f Li(Scene* scene, Sampler* sampler, const Ray& ray) const = 0;
};

class GeometryIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build GeometryIntegrator!" << endl;
	}

	Color3f Li(Scene* scene, Sampler* sampler, const Ray& ray) const;
};

class BaseColorIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build BaseColorIntegrator!" << endl;
	}

	Color3f Li(Scene* scene, Sampler* sampler, const Ray& ray) const;
};

class PathIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build PathIntegrator!" << endl;
	}

	Color3f Li(Scene* scene, Sampler* sampler, const Ray& ray) const;

private:
	Vector3f sampleLd(Scene* scene, Sampler* sampler, const Intersection& its, const Vector3f& wo) const;

};

}