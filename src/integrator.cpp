#pragma once

#include <pt/color.h>
#include <pt/ray.h>
#include <pt/scene.h>
#include <pt/shape.h>
#include <pt/material.h>
#include <pt/integrator.h>
#include <pt/sampler.h>
#include <pt/light.h>
#include <pt/camera.h>
	
namespace pt {

inline float powerHeuristic(float f, float g) {
	float f2 = f * f, g2 = g * g;
	return f2 / (f2 + g2);
}

Vector3f GeometryIntegrator::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Ray ray = scene->getCamera()->sampleRay(pixelSample);
	Intersection its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f n = its.n;
		n = n.cwiseAbs();
		return n;
	}
	else
		return Vector3f(0);
}

Vector3f BaseColorIntegrator::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Ray ray = scene->getCamera()->sampleRay(pixelSample);
	Intersection its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f c = its.getMaterial()->getBaseColor(its.uv);
		return c;
	}
	else
		return Vector3f(0);
}

Vector3f PathIntegrator::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Ray ray = scene->getCamera()->sampleRay(pixelSample);
	Vector3f L(0.0), accThroughput(1.0);
	float brdfPdf;

	for(int bounce = 0; bounce < 16; bounce++) {
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
				float light_pdf = light->pdfLi(its, ray);
				light_pdf *= scene->getLightSelector()->pdf(light); // select pdf
				float misWeight = powerHeuristic(brdfPdf, light_pdf);
				L += misWeight * accThroughput.cwiseProduct(Le); // brdf mis
			}
		}

		// sample light
		Vector3f Ld = sampleLd(scene, sampler, its, wo);
		L += accThroughput.cwiseProduct(Ld);

		// sample BRDF
		BRDFSample bs = its.sampleBRDF(wo, sampler->sample1D(), sampler->sample2D());
		if (bs.f.squaredNorm() == 0.0f || bs.pdf == 0.0f) break; // no enerage
		Vector3f throughput =  bs.f * its.n.dot(bs.wi) / bs.pdf;
		accThroughput = accThroughput.cwiseProduct(throughput);
		brdfPdf = bs.pdf;

		// new ray
		ray = its.genRay(bs.wi);

		// possibly terminate the path with Russian roulette
		if (accThroughput.maxCoeff() < 1.0f && bounce > 1) {
			float q = std::max(0.0f, 1.0f - accThroughput.maxCoeff());
			if (sampler->sample1D() < q) break;
			accThroughput /= (1.0f - q);
		}
	}

	return L;
}

Vector3f PathIntegrator::sampleLd(Scene* scene, Sampler* sampler, const Intersection& surfIts, const Vector3f& wo) const {
	const std::vector<AreaLight*>& lights = scene->getLights();
	int nLights = lights.size();
	if (lights.empty())
		return Vector3f(0.0);

	// uniformly select a light source
	AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
	float selectPdf = scene->getLightSelector()->pdf(light);

	// sample a point on the light source (sample a triangle)
	LightLiSample lightIts = light->sampleLi(surfIts, sampler->sample2D());
	if (lightIts.pdfDir == 0.0f)
		return Vector3f(0.0);

	// visibility test
	if (!scene->unocculded(surfIts.p, lightIts.p, surfIts.n, lightIts.n))
		return Vector3f(0.0);
	Vector3f& wi = lightIts.wi;
	Vector3f& Le = lightIts.L;

	// phong BRDF
	Vector3f f = surfIts.BRDF(wo, wi);
	float cosTheta = surfIts.n.dot(wi);

	// light mis
	float brdf_pdf = surfIts.pdfBRDF(wo, wi);
	float light_pdf = lightIts.pdfDir * selectPdf;
	float misWeight = powerHeuristic(light_pdf, brdf_pdf);

	return misWeight * f.cwiseProduct(Le) * cosTheta / light_pdf;
}

}