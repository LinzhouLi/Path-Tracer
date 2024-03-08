#pragma once

#include <pt/common.h>
#include <pt/ray.h>

namespace pt {

struct LightLiSample {
	Vector3f L;
	Vector3f wi; // incident light direction
	Vector3f p;
	Vector3f n;
	float pdfArea; // measure in area
	float pdfDir; // measure in solid angle
};

struct LightLeSample {
	Vector3f L;
	Ray ray; // light emitted from the light source
	Vector3f n;
	float pdfArea; // measure in area
	float pdfDir; // measure in direction
};

class AreaLight {
public:
	AreaLight(Triangle* shape, const Vector3f& lemit);

	// surface normal, ray direction
	Vector3f L(const Vector3f& n, const Vector3f& w) const;

	Vector3f power() const { return m_area * m_lemit * M_PI; }

	LightLiSample sampleLi(const Intersection& surfIts, const Vector2f& u) const;

	LightLeSample sampleLe(const Vector2f& u1, const Vector2f& u2) const;

	float pdfLi(const Intersection& lightIts, const Ray& ray) const;

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