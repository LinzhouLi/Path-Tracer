#include <pt/light.h>
#include <pt/shape.h>
#include <pt/scene.h>
#include <pt/sampler.h>
#include <pt/material.h>

namespace pt {

AreaLight::AreaLight(Triangle* shape, const Vector3f& lemit) : m_shape(shape), m_lemit(lemit) {
	m_area = shape->surfaceArea();
}

Vector3f AreaLight::L(const Vector3f& n, const Vector3f& w) const {
	return n.dot(w) > 0.0 ? m_lemit : Vector3f(0.0);
}

TriangleSample AreaLight::sampleLi(const Vector2f& u) const {
	return m_shape->sample(u);
}

Vector3f Scene::uniformSampleLights(const Intersection& its, Sampler* sampler) {
	int nLights = m_lights.size();
	if (m_lights.empty())
		return Vector3f(0.0);

	// uniformly select a light source
	int lightIndex = std::min(int(sampler->sample1D() * nLights), nLights - 1);
	float selectPdf = 1.0 / nLights;
	AreaLight* light = m_lights[lightIndex];

	// sample a point on the light source (sample a triangle)
	TriangleSample light_its = light->sampleLi(sampler->sample2D());

	// visibility test
	if (!unocculded(its.p, light_its.p, its.n, light_its.n))
		return Vector3f(0.0);

	// ray from the intersection point to the light source
	Vector3f wi = light_its.p - its.p;
	float distance = wi.norm();
	if (distance == 0.0)
		return Vector3f(0.0);
	wi /= distance;

	// light radiance
	Vector3f Li = light->L(light_its.n, -wi);
	if (Li.squaredNorm() == 0.0)
		return Vector3f(0.0);

	// light solid angle pdf
	float solidAnglePdf = light_its.pdf * distance / light_its.n.dot(-wi); // pdf_area * r^2 / cos_theta

	// f
	Vector3f r = getMaterial(its.getShape()->getMaterialId())->getBaseColor(its.uv);
	float cos = its.n.dot(wi);
	Vector3f f = cos > 0.0 ? r * INV_PI : Vector3f(0.0);

	return f.cwiseProduct(Li) * cos / (solidAnglePdf * selectPdf);
}

}