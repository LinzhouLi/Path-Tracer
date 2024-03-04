#pragma once

#include <pt/color.h>
#include <pt/ray.h>
#include <pt/scene.h>
#include <pt/shape.h>
#include <pt/material.h>
#include <pt/integrator.h>
#include <pt/sampler.h>
#include <pt/light.h>

namespace pt {

Color3f GeometryIntegrator::Li(Scene* scene, Sampler* sampler, const Ray& ray) const {
	Intersection its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f n = its.n;
		n = n.cwiseAbs();
		return Color3f(n.x(), n.y(), n.z());
	}
	else
		return Color3f(0, 0, 0);
}

Color3f BaseColorIntegrator::Li(Scene* scene, Sampler* sampler, const Ray& ray) const {
	Intersection its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f c = its.getMaterial()->getBaseColor(its.uv);
		return Color3f(c.x(), c.y(), c.z());
	}
	else
		return Color3f(0, 0, 0);
}

Color3f PathIntegrator::Li(Scene* scene, Sampler* sampler, const Ray& ray) const {
	Vector3f L(0.0), beta(1.0);

	for(int bounce = 0; bounce < 1; bounce++) {
		Intersection its;
		bool hit = scene->rayIntersect(ray, its);
		if (!hit) break;

		if (bounce == 0) {
			L += beta.cwiseProduct(its.Le(-ray.dir));
		}

		// sample light
		Vector3f Ld = sampleLd(scene, sampler, its, -ray.dir);
		L += beta.cwiseProduct(Ld);
	}

	return Color3f(L.x(), L.y(), L.z());
}

Vector3f PathIntegrator::sampleLd(Scene* scene, Sampler* sampler, const Intersection& its, const Vector3f& wo) const {
	const std::vector<AreaLight*>& lights = scene->getLights();
	int nLights = lights.size();
	if (lights.empty())
		return Vector3f(0.0);

	// uniformly select a light source
	int lightIndex = std::min(int(sampler->sample1D() * nLights), nLights - 1);
	float selectPdf = 1.0 / nLights;
	AreaLight* light = lights[lightIndex];

	// sample a point on the light source (sample a triangle)
	TriangleSample light_its = light->sampleLi(sampler->sample2D());

	// visibility test
	if (!scene->unocculded(its.p, light_its.p, its.n, light_its.n))
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

	// phong BRDF
	Vector3f f = its.BRDF(wo, wi);
	float cosTheta = its.n.dot(wi);

	return f.cwiseProduct(Li) * cosTheta / (solidAnglePdf * selectPdf);
}

}