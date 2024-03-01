#pragma once

#include <pt/color.h>
#include <pt/ray.h>
#include <pt/scene.h>
#include <pt/shape.h>
#include <pt/material.h>
#include <pt/integrator.h>

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
		Material* m = scene->getMaterial(its.getShape()->getMaterialId());
		Vector3f c = m->getBaseColor(its.uv);
		return Color3f(c.x(), c.y(), c.z());
	}
	else
		return Color3f(0, 0, 0);
}

Color3f PathIntegrator::Li(Scene* scene, Sampler* sampler, const Ray& ray) const {
	Vector3f L(0.0), beta(1.0);
	bool specularBounce = false;

	for(int bounce = 0; bounce < 1; bounce++) {
		Intersection its;
		bool hit = scene->rayIntersect(ray, its);
		if (!hit) break;

		if (bounce == 0 || specularBounce) {
			L += beta.cwiseProduct(its.Le(-ray.dir));
		}

		// sample light
		Vector3f Ld = scene->uniformSampleLights(its, sampler);
		L += beta.cwiseProduct(Ld);
	}

	return Color3f(L.x(), L.y(), L.z());
}

}