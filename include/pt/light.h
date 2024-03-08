#pragma once

#include <pt/common.h>

namespace pt {

struct LightSample {
	LightSample(
		const Vector3f& Le_ = Vector3f(), const Vector3f& wi_ = Vector3f(), 
		const Vector3f& p_ = Vector3f(), const Vector3f& n_ = Vector3f(), float pdf_ = 0.0f) :
		Le(Le_), wi(wi_), p(p_), n(n_), pdf(pdf_) { }

	Vector3f Le;
	Vector3f wi;
	Vector3f p;
	Vector3f n;
	float pdf; // area measure
};

class AreaLight {
public:
	AreaLight(Triangle* shape, const Vector3f& lemit);

	// surface normal, ray direction
	Vector3f L(const Vector3f& n, const Vector3f& w) const;

	Vector3f power() const { return m_area * m_lemit * M_PI; }

	LightSample sampleLi(const Intersection& surfIts, const Vector2f& u) const;

	float pdf(const Intersection& lightIts, const Ray& ray) const;

private:
	Triangle* m_shape;
	Vector3f m_lemit;
	float m_area;

};


class UniformLightSelector {
public:
	UniformLightSelector(std::vector<AreaLight*>* lights) : m_lights(lights) { }

	AreaLight* select(float u) const {
		return m_lights->at(static_cast<int>(u * m_lights->size()));
	}

	float pdf(const AreaLight* light) const {
		return 1.0f / m_lights->size();
	}

private:
	std::vector<AreaLight*>* m_lights;

};

}