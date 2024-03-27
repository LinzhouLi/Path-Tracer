#include <pt/bdpt2.h>
#include <pt/scene.h>
#include <pt/sampler.h>
#include <pt/ray.h>
#include <pt/material.h>
#include <pt/light.h>
#include <pt/camera.h>
#include <pt/block.h>

namespace pt {

	const int max_recursive_depth = 6;

void BDPTIntegrator2::connectCamera(const BDPTVertex& vertex, Camera* camera, Scene* scene, Sampler* sampler) const {
	if (vertex.depth >= max_recursive_depth)
		return;

	auto pixel = camera->project(vertex.its.p);
	CameraLiSample cs = camera->sampleLi(vertex.its, sampler->sample2D());
	if (!pixel.has_value() || !scene->unocculded(vertex.its.p, cs.p, vertex.its.ng)) return;

	Vector3f radiance(0.0);
	if (cs.pdfDir != 0.0f && cs.L.squaredNorm() != 0.0f) {
		radiance = vertex.throughput
			.cwiseProduct(vertex.its.BRDF(vertex.wi, cs.wi))
			.cwiseProduct(cs.L / cs.pdfDir) * absDot(vertex.its.n, cs.wi);
	}

	if (radiance.squaredNorm() != 0.0) {
		float invSqrLen = 1.0f / (vertex.its.p - cs.p).squaredNorm();
		float lightvert_pdfA = camera->pdfLe(Ray(Vector3f(0.0f), -cs.wi)) * absDot(vertex.its.n, cs.wi) * invSqrLen;
		float bsdf_rev_pdfw = vertex.its.pdfBRDF(cs.wi, vertex.wi) * vertex.rr;
		double mis0 = (vertex.vcm + vertex.vc * MIS(bsdf_rev_pdfw)) * MIS(lightvert_pdfA);
		float weight = (float)(1.0f / (1.0f + mis0));
		radiance *= weight;
	}
	m_splatBlock->addSplat(pixel.value(), radiance);
}

Vector3f BDPTIntegrator2::connectLight(const BDPTVertex& vertex, AreaLight* light, Scene* scene, Sampler* sampler) const {
	if (vertex.depth >= max_recursive_depth)
		return 0.0f;

	// select light
	//AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
	//float selectPdf = scene->getLightSelector()->pdf(light);

	// sample incident ray on the light shape
	LightLiSample ls = light->sampleLi(vertex.its, sampler->sample2D());

	Vector3f radiance(0.0);
	if (
		ls.pdfDir != 0.0f && ls.L.squaredNorm() != 0.0f &&
		scene->unocculded(vertex.its.p, ls.p, vertex.its.ng, ls.n) // visibility test
		) {
		radiance = vertex.throughput
			.cwiseProduct(vertex.its.BRDF(vertex.wi, ls.wi))
			.cwiseProduct(ls.L / ls.pdfDir) * absDot(vertex.its.n, ls.wi);
	}

	if (radiance.squaredNorm() != 0.0f) {
		float eye_bsdf_pdfw = vertex.its.pdfBRDF(vertex.wi, ls.wi) * vertex.rr;
		float eye_bsdf_rev_pdfw = vertex.its.pdfBRDF(ls.wi, vertex.wi) * vertex.rr;

		float emissionPdf = INV_TWOPI * ls.pdfArea;
		double mis0 = MIS(eye_bsdf_pdfw / ls.pdfDir);
		double mis1 = MIS(absDot(vertex.its.n, ls.wi) * emissionPdf / (absDot(ls.n, ls.wi) * ls.pdfDir)) *
			(vertex.vcm + vertex.vc * MIS(eye_bsdf_rev_pdfw));

		float weight = (float)(1.0f / (mis0 + mis1 + 1.0f));

		radiance *= weight;
	}
	return radiance;
}

Vector3f BDPTIntegrator2::connectVertices(const BDPTVertex& p0, const BDPTVertex& p1, Scene* scene, Sampler* sampler) const {
	if (p0.depth + p1.depth >= max_recursive_depth)
		return 0.0f;

	if (!scene->unocculded(p0.its.p, p1.its.p, p0.its.ng, p1.its.ng))
		return 0.0f;

	Vector3f delta = p0.its.p - p1.its.p;
	float invDistcSqr = 1.0f / delta.squaredNorm();
	Vector3f n_delta = delta * sqrt(invDistcSqr);

	float cosAtP0 = absDot(p0.its.n, n_delta);
	float cosAtP1 = absDot(p1.its.n, n_delta);
	const Vector3f g = p1.its.BRDF(p1.wi, n_delta).cwiseProduct(p0.its.BRDF(p0.wi, -n_delta)) * invDistcSqr;
	if (g.squaredNorm() == 0.0f) return 0.0f;

	float p0_bsdf_pdfw = p0.its.pdfBRDF(p0.wi, -n_delta) * p0.rr;
	float p0_bsdf_rev_pdfw = p0.its.pdfBRDF(-n_delta, p0.wi) * p0.rr;
	float p1_bsdf_pdfw = p1.its.pdfBRDF(p1.wi, n_delta) * p1.rr;
	float p1_bsdf_rev_pdfw = p1.its.pdfBRDF(n_delta, p1.wi) * p1.rr;

	float p0_a = p1_bsdf_pdfw * cosAtP0 * invDistcSqr;
	float p1_a = p0_bsdf_pdfw * cosAtP1 * invDistcSqr;

	const double mis_0 = MIS(p0_a) * (p0.vcm + p0.vc * MIS(p0_bsdf_rev_pdfw));
	const double mis_1 = MIS(p1_a) * (p1.vcm + p1.vc * MIS(p1_bsdf_rev_pdfw));

	const auto weight = (float)(1.0f / (mis_0 + 1.0f + mis_1));

	return p0.throughput.cwiseProduct(p1.throughput).cwiseProduct(g) * weight;
}

Vector3f BDPTIntegrator2::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Camera* camera = scene->getCamera();

	Vector3f radiance(0.0f);

	// select light
	AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
	float selectPdf = scene->getLightSelector()->pdf(light);

	LightLeSample ls = light->sampleLe(sampler->sample2D(), sampler->sample2D());
	float cosAtLight = absDot(ls.n, ls.ray.dir);
	float light_emission_pdf = ls.pdfArea * ls.pdfDir;
	float light_pdfa = ls.pdfArea;

	//-----------------------------------------------------------------------------------------------------
	// Trace light path from light source
	std::vector<BDPTVertex> light_path;
	Ray ray = ls.ray;
	double vc = MIS(cosAtLight / light_emission_pdf);
	double vcm = MIS(light_pdfa / light_emission_pdf);
	Vector3f throughput = ls.L * cosAtLight / (light_emission_pdf * selectPdf);
	float rr = 1.0f;
	while (light_path.size() < max_recursive_depth) {
		BDPTVertex vert;
		bool hit = scene->rayIntersect(ray, vert.its);
		if (!hit) break;

		float distSqr = (vert.its.p - ray.org).squaredNorm();
		float cosIn = absDot(ray.dir, vert.its.n);
		if (light_path.size() > 0 || light_path.size() == 0)
			vcm *= MIS(distSqr);
		vcm /= MIS(cosIn);
		vc /= MIS(cosIn);

		rr = 1.0f;
		//if (throughput.GetIntensity() < 0.01f)
		//	rr = 0.5f;

		vert.wi = -ray.dir;
		vert.throughput = throughput;
		vert.vcm = vcm;
		vert.vc = vc;
		vert.rr = rr;
		vert.depth = (unsigned)(light_path.size() + 1);

		light_path.push_back(vert);

		//-----------------------------------------------------------------------------------------------------
		// Path evaluation: light tracing
		connectCamera(vert, camera, scene, sampler);

		// russian roulette
		if (sampler->sample1D() > rr) break;

		BRDFSample bs = vert.its.sampleBRDF(vert.wi, sampler->sample1D(), sampler->sample2D());
		float bsdf_pdf = bs.pdf * rr;
		vert.wo = bs.wi;

		if (bs.pdf == 0.0f) break;
		float cosOut = absDot(vert.wo, vert.its.n);
		throughput = throughput.cwiseProduct(bs.f) / bsdf_pdf;

		if (throughput.squaredNorm() == 0.0f) break;

		float rev_bsdf_pdfw = vert.its.pdfBRDF(vert.wo, vert.wi) * rr;
		vc = MIS(cosOut / bsdf_pdf) * (MIS(rev_bsdf_pdfw) * vc + vcm);
		vcm = MIS(1.0f / bsdf_pdf);

		ray = vert.its.genRay(vert.wo);
	}

	//-----------------------------------------------------------------------------------------------------
	// Trace light path from camera point
	int lps = light_path.size();
	ray = camera->sampleRay(pixelSample);
	throughput = 1.0f;
	int light_path_len = 0;
	vc = 0.0f;
	vcm = MIS(1.0 / camera->pdfLe(ray));
	rr = 1.0f;
	while (light_path_len <= max_recursive_depth) {
		BDPTVertex vert;
		vert.depth = light_path_len;
		bool hit = scene->rayIntersect(ray, vert.its);
		if (!hit) break; 

		float distSqr = (vert.its.p - ray.org).squaredNorm();
		float cosIn = absDot(ray.dir, vert.its.n);
		vcm *= MIS(distSqr);
		vcm /= MIS(cosIn);
		vc /= MIS(cosIn);

		//-----------------------------------------------------------------------------------------------------
		// Path evaluation: it hits a light source
		const AreaLight* hitLight = vert.its.getLight();
		if (hitLight) {
			if (vert.depth > 0 && vert.depth <= max_recursive_depth) {
				float emissionPdf = INV_TWOPI * light->pdfArea(); // refactor
				float directPdfA = light->pdfArea();
				Vector3f li = hitLight->L(vert.its.n, -ray.dir).cwiseProduct(throughput) / selectPdf;
				radiance += li / (float)(1.0f + MIS(directPdfA) * vcm + MIS(emissionPdf) * vc);
			}
			else if (vert.depth == 0)
				radiance += vert.its.Le(-ray.dir) / selectPdf;
		}

		rr = std::min(1.0f, throughput.maxCoeff());

		vert.wi = -ray.dir;
		vert.throughput = throughput;
		vert.vc = vc;
		vert.vcm = vcm;
		vert.rr = rr;

		//-----------------------------------------------------------------------------------------------------
		// Path evaluation: connect light sample first
		radiance += connectLight(vert, light, scene, sampler) / selectPdf;

		//-----------------------------------------------------------------------------------------------------
		// Path evaluation: connect vertices
		for (unsigned j = 0; j < lps; ++j)
			radiance += connectVertices(light_path[j], vert, scene, sampler);

		++light_path_len;

		// Russian Roulette
		if (sampler->sample1D() > rr) break;

		BRDFSample bs = vert.its.sampleBRDF(vert.wi, sampler->sample1D(), sampler->sample2D());
		float bsdf_pdf = bs.pdf * rr;
		vert.wo = bs.wi;

		if (bsdf_pdf == 0.0f) break;
		const auto cosOut = absDot(vert.wo, vert.its.n);
		throughput = throughput.cwiseProduct(bs.f) / bsdf_pdf;

		if (throughput.squaredNorm() == 0.0f) break;

		float rev_bsdf_pdfw = vert.its.pdfBRDF(vert.wo, vert.wi) * rr;
		vc = MIS(cosOut / bsdf_pdf) * (MIS(rev_bsdf_pdfw) * vc + vcm);
		vcm = MIS(1.0f / bsdf_pdf);

		ray = vert.its.genRay(vert.wo);
	}

	return radiance;
}

}