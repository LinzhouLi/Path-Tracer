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

inline float powerHeuristic(float f, float g) {
	float f2 = f * f, g2 = g * g;
	return f2 / (f2 + g2);
}

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

Color3f PathIntegrator::Li(Scene* scene, Sampler* sampler, const Ray& ray_) const {
	Vector3f L(0.0), accThroughput(1.0);
	Ray ray(ray_);
	float brdf_pdf;

	for(int bounce = 0; bounce < 1; bounce++) {
		Intersection its;
		bool hit = scene->rayIntersect(ray, its);
		if (!hit) break;

		Vector3f wo = -ray.dir;

		// hit light
		const AreaLight* light = its.getLight();
		if (light) {
			Vector3f Le = its.Le(wo);
			if (bounce == 0) L += accThroughput.cwiseProduct(Le);
			else {
				float light_pdf = light->pdf(its, ray);
				light_pdf *= 1.0 / scene->getLights().size(); // select pdf
				float misWeight = powerHeuristic(brdf_pdf, light_pdf);
				L += misWeight * accThroughput.cwiseProduct(Le);
			}
		}

		// sample light
		Vector3f Ld = sampleLd(scene, sampler, its, wo);
		L += accThroughput.cwiseProduct(Ld);

		// sample BRDF
		BRDFSample bs = its.sampleBRDF(wo, sampler->sample1D(), sampler->sample2D());
		if (bs.f.squaredNorm() == 0.0f || bs.pdf == 0.0f) break;
		Vector3f throughput =  bs.f * its.n.dot(bs.wi) / bs.pdf;
		accThroughput = accThroughput.cwiseProduct(throughput);
		brdf_pdf = bs.pdf;

		// new ray
		ray = its.genRay(bs.wi);
	}

	return Color3f(L.x(), L.y(), L.z());
}

Vector3f PathIntegrator::sampleLd(Scene* scene, Sampler* sampler, const Intersection& surfIts, const Vector3f& wo) const {
	const std::vector<AreaLight*>& lights = scene->getLights();
	int nLights = lights.size();
	if (lights.empty())
		return Vector3f(0.0);

	// uniformly select a light source
	int lightIndex = std::min(int(sampler->sample1D() * nLights), nLights - 1);
	float selectPdf = 1.0 / nLights;
	AreaLight* light = lights[lightIndex];

	// sample a point on the light source (sample a triangle)
	LightSample lightIts = light->sampleLi(surfIts, sampler->sample2D());
	if (lightIts.pdf == 0.0f)
		return Vector3f(0.0);

	// visibility test
	if (!scene->unocculded(surfIts.p, surfIts.p, lightIts.n, lightIts.n))
		return Vector3f(0.0);
	Vector3f& wi = lightIts.wi;
	Vector3f& Le = lightIts.Le;

	// phong BRDF
	Vector3f f = surfIts.BRDF(wo, wi);
	float cosTheta = surfIts.n.dot(wi);

	// light mis
	float brdf_pdf = surfIts.pdfBRDF(wo, wi);
	float light_pdf = lightIts.pdf * selectPdf;
	float misWeight = powerHeuristic(light_pdf, brdf_pdf);

	return misWeight * f.cwiseProduct(Le) * cosTheta / light_pdf;
}

}