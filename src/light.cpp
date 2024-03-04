#include <pt/light.h>
#include <pt/shape.h>
#include <pt/scene.h>
#include <pt/sampler.h>
#include <pt/material.h>
#include <pt/ray.h>

namespace pt {

AreaLight::AreaLight(Triangle* shape, const Vector3f& lemit) : m_shape(shape), m_lemit(lemit) {
	m_area = shape->surfaceArea();
}

Vector3f AreaLight::L(const Vector3f& n, const Vector3f& w) const {
	return n.dot(w) > 0.0 ? m_lemit : Vector3f(0.0);
}

LightSample AreaLight::sampleLi(const Intersection& surfIts, const Vector2f& u) const {
	TriangleSample lightIts = m_shape->sample(u);
	Vector3f wi = lightIts.p - surfIts.p; // ray from the intersection point to the light source

	float distance = wi.norm();
	wi /= distance; // normalize wi
	float cos_lw = lightIts.n.dot(-wi);
	float cos_sw = surfIts.n.dot(wi);

	if (cos_lw > 0.0 && cos_sw > 0.0 && distance > 0.0) {
		float solidAnglePdf = lightIts.pdf * distance * distance / cos_lw; // pdf_area * r^2 / cos_theta
		return LightSample(m_lemit, wi, lightIts.p, lightIts.n, solidAnglePdf);
	}
	else
		return LightSample();
}

float AreaLight::pdf(const Intersection& lightIts, const Ray& ray) const {
	float distance = (lightIts.p - ray.org).norm();
	float cos_lw = lightIts.n.dot(-ray.dir);
	return m_shape->pdf() * distance * distance / cos_lw;
}

}