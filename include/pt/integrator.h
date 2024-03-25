#pragma once

#include <pt/common.h>

namespace pt {

struct LightLiSample;

class Integrator {
public:
	virtual Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) = 0;

	void setSplatBlock(ImageBlock* block) { m_splatBlock = block; }

	virtual std::string toString() const = 0;

protected:
	ImageBlock* m_splatBlock = nullptr;
};

class GeometryIntegrator : public Integrator {
public:
	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

	std::string toString() const {
		return tfm::format("GeometryIntegrator[]");
	}
};

class BaseColorIntegrator : public Integrator {
public:
	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

	std::string toString() const {
		return tfm::format("BaseColorIntegrator[]");
	}
};

class PathIntegrator : public Integrator {
public:
	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

	std::string toString() const {
		return tfm::format("PathIntegrator[]");
	}

private:
	Vector3f sampleLd(Scene* scene, Sampler* sampler, const Intersection& its, const Vector3f& wo) const;

};

}