#pragma once

#include <pt/common.h>

namespace pt {

struct LightLiSample;

class Integrator {
public:
	virtual void preprocess(Scene* scene) { }

	virtual Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) = 0;

	void setResultBlock(ImageBlock* block) { m_block = block; }

protected:
	ImageBlock* m_block;
};

class GeometryIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build GeometryIntegrator!" << endl;
	}

	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);
};

class BaseColorIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build BaseColorIntegrator!" << endl;
	}

	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);
};

class PathIntegrator : public Integrator {
public:
	void preprocess(Scene* scene) {
		cout << "Build PathIntegrator!" << endl;
	}

	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

private:
	Vector3f sampleLd(Scene* scene, Sampler* sampler, const Intersection& its, const Vector3f& wo) const;

};

}