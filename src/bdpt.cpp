#include <pt/bdpt.h>
#include <pt/scene.h>
#include <pt/sampler.h>
#include <pt/ray.h>
#include <pt/material.h>
#include <pt/light.h>
#include <pt/camera.h>
#include <pt/block.h>

namespace pt {

float correctShadingNormal(const Intersection& its, const Vector3f& wo, const Vector3f& wi, TransportMode mode) {
	if (mode == TransportMode::Radiance) {
		return 1.0f;
	}
	else {
		float num = its.n.dot(wo) * its.ng.dot(wi); // cos(wo, ns) * cos(wi, ng)
		float denom = its.ng.dot(wo) * its.n.dot(wi); // cos(wo, ng) * cos(wi, ns)
		if (denom == 0.0f) return 0.0f;
		else return num / denom;
	}
}

float G(const Vertex& a, const Vertex& b) {
	Vector3f ab = b.its.p - a.its.p;
	float dist = ab.norm();
	if (dist == 0.0f) return 0.0f;
	ab /= dist;
	return a.its.n.dot(ab) * b.its.n.dot(-ab) / (dist * dist);
}


inline Vertex Vertex::createFromLight(const AreaLight* light, const Vector3f& position, const Vector3f& normal, const Vector3f& Le, float pdf) {
	Vertex v;
	v.type = VertexType::Light;
	v.its.p = position;
	v.its.n = v.its.ng = normal;
	v.pdfFwd = pdf;
	v.light = light;
	v.beta = Le;
	return v;
}

inline Vertex Vertex::createFromCamera(const Camera* camera, const Vector3f& position, const Vector3f& beta) {
	Vertex v;
	v.type = VertexType::Camera;
	v.beta = beta;
	v.its.p = position;
	return v;
}

inline Vertex Vertex::createFromSurface(const Intersection& its, const Vector3f& beta, float pdf, const Vertex& preVertex) {
	Vertex v;
	v.type = VertexType::Surface;
	v.its = its;
	v.beta = beta;
	v.pdfFwd = preVertex.convertPdfDensity(pdf, v);
	return v;
}

float Vertex::convertPdfDensity(float pdf, const Vertex& nextVertex) const {
	Vector3f w = nextVertex.its.p - this->its.p;
	float dist = w.norm();
	if (dist == 0.0f) return 0.0f;
	if (nextVertex.type != VertexType::Camera) {
		w /= dist;
		pdf *= nextVertex.its.ng.dot(-w); // why ng?
	}
	return pdf / (dist * dist);
}

Vector3f Vertex::Le(const Vertex& v) const {
	if (its.getShape() && its.getShape()->getLight()) {
		Vector3f w = v.its.p - this->its.p;
		float dist = w.norm();
		if (dist == 0.0f) return Vector3f(0.0f);
		w /= dist;
		return its.Le(w);
	}
	else return Vector3f(0.0f);
}

Vector3f Vertex::BRDF(const Vertex& preVertex, const Vertex& nextVertex, TransportMode mode) const {
	if (this->type != VertexType::Surface) return Vector3f(0.0f);

	Vector3f wi = nextVertex.its.p - this->its.p;
	Vector3f wo = preVertex.its.p - this->its.p;

	float dist = wi.norm();
	if (dist == 0.0f) return Vector3f(0.0f);
	wi /= dist;
	wo.normalize();

	return its.BRDF(wo, wi) * correctShadingNormal(its, wo, wi, mode);
}


int BDPTIntegrator::randomWalk(
	Scene* scene, Sampler* sampler, Vertex* path, Ray ray,
	Vector3f beta, float pdf, int maxDepth, TransportMode mode
) const {
	if (maxDepth == 0) return 0;

	int bounce;
	float pdfFwd = pdf, pdfRev = 0;
	for (bounce = 0; bounce < maxDepth; bounce++) {
		Intersection its;
		bool hit = scene->rayIntersect(ray, its);
		if (!hit) break;

		Vertex& vertex = path[bounce];
		Vertex& preVertex = path[bounce - 1];

		// create surface vertex
		vertex = Vertex::createFromSurface(its, beta, pdfFwd, preVertex);

		// sample BRDF
		Vector3f wo = -ray.dir;
		BRDFSample bs = its.sampleBRDF(wo, sampler->sample1D(), sampler->sample2D());
		if (bs.f.squaredNorm() == 0.0f || bs.pdf == 0.0f) break; // no enerage

		// accumulate beta (throughput)
		Vector3f throughput = bs.f * its.n.dot(bs.wi) / bs.pdf;
		beta = beta.cwiseProduct(throughput);
		beta *= correctShadingNormal(its, wo, bs.wi, mode);

		// record pdf
		pdfFwd = bs.pdf;
		pdfRev = its.pdfBRDF(bs.wi, wo); // reverse wi and wo

		// compute reverse area density at prev vertex
		preVertex.pdfRev = vertex.convertPdfDensity(pdfRev, preVertex);

		// new ray
		ray = its.genRay(bs.wi);
	}
	return bounce;
}

int BDPTIntegrator::generateCameraSubpath(Scene* scene, Sampler* sampler, Vertex* path, const Vector2f& pixelSample, int maxDepth) const {
	if (maxDepth == 0) return 0;

	float beta = 1.0f;
	Ray ray = scene->getCamera()->sampleRay(pixelSample);
	path[0] = Vertex::createFromCamera(scene->getCamera(), ray.org, beta);
	int numVertices = randomWalk(
		scene, sampler, path + 1, ray,
		beta, 1.0f, maxDepth - 1, TransportMode::Radiance
	);
	return numVertices + 1;
}

int BDPTIntegrator::generateLightSubpath(Scene* scene, Sampler* sampler, Vertex* path, int maxDepth) const {
	if (maxDepth == 0) return 0;

	// select light
	AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
	float selectPdf = scene->getLightSelector()->pdf(light);

	// sample outgoing ray from light
	LightLeSample ls = light->sampleLe(sampler->sample2D(), sampler->sample2D());
	if (ls.pdfArea == 0.0f || ls.pdfDir == 0.0f || ls.L.squaredNorm() == 0.0f) return 0;

	// create first vertex
	Vector3f beta = ls.L * ls.n.dot(ls.ray.dir) / (ls.pdfArea * ls.pdfDir * selectPdf); // why dot?
	path[0] = Vertex::createFromLight(light, ls.ray.org, ls.n, ls.L, ls.pdfArea * selectPdf);

	// sample other vertices
	int numVertices = randomWalk(
		scene, sampler, path + 1, ls.ray, 
		beta, ls.pdfDir, maxDepth - 1, TransportMode::Importance
	);
	return numVertices + 1;
}

Vector3f BDPTIntegrator::connectLightPath(
	Scene* scene, Sampler* sampler,
	Vertex* lightVertices, Vertex* cameraVertices,
	int s, int t, Vector2f& pixelSample
) const {
	Vector3f L(0.0);
	Vertex sampledVertex;

	// connect vertices
	if (s == 0) { // use full camera path, has energy only when the last vertex emits light
		const Vertex& vt = cameraVertices[t - 1];
		L = vt.Le(cameraVertices[t - 2]).cwiseProduct(vt.beta);
	}
	else if (t == 1) { // sample a point on the camera and connect it to the light subpath

	}
	else if (s == 1) { // sample a point on a light and connect it to the camera subpath. why resample?
		const Vertex& vt = cameraVertices[t - 1];
		const Vertex& vt_prev = cameraVertices[t - 2];
		// select light
		AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
		float selectPdf = scene->getLightSelector()->pdf(light);

		// sample incident ray on the light shape
		LightLiSample ls = light->sampleLi(vt.its, sampler->sample2D());
		if (
			ls.pdfArea != 0.0f && ls.pdfDir != 0.0f && ls.L.squaredNorm() != 0.0f &&
			scene->unocculded(vt.its.p, ls.p, vt.its.n, ls.n)
		) {
			float pdfArea = ls.pdfArea * selectPdf;
			sampledVertex = Vertex::createFromLight(light, ls.p, ls.n, ls.L / pdfArea, pdfArea);
			L = vt.beta.cwiseProduct(vt.BRDF(vt_prev, sampledVertex, TransportMode::Radiance))
				.cwiseProduct(sampledVertex.beta) * vt.its.n.dot(ls.wi);
		}
	}
	else { // general cases
		const Vertex& vs = lightVertices[s - 1];
		const Vertex& vt = cameraVertices[t - 1];
		if (scene->unocculded(vs.its.p, vt.its.p, vs.its.n, vt.its.n)) {
			const Vertex& vs_prev = lightVertices[s - 2];
			const Vertex& vt_prev = cameraVertices[t - 2];
			L = vs.beta.cwiseProduct(vs.BRDF(vs_prev, vt, TransportMode::Importance))
				.cwiseProduct(vt.BRDF(vt_prev, vs, TransportMode::Radiance))
				.cwiseProduct(vt.beta);
			if (L.squaredNorm() != 0.0f) L *= G(vs, vt);
		}
	}
	return L;
}

Vector3f BDPTIntegrator::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Vector3f L(0.0);

	// generate subpaths
	Vertex cameraVertices[MaxDepth + 2], lightVertices[MaxDepth + 1];
	int numCameraVs = generateCameraSubpath(scene, sampler, cameraVertices, pixelSample, MaxDepth + 2);
	int numLightVs = generateLightSubpath(scene, sampler, lightVertices, MaxDepth + 1);
	if (numCameraVs < 2) return L; // at least one segment on camera subpath

	// connect subpaths and compute contribution
	for (int t = 2; t <= numCameraVs; t++) {
		for (int s = 0; s <= numLightVs; s++) {
			int depth = t + s - 2;
			if ((s == 1 && t == 1) || depth < 0 || depth > MaxDepth) continue;

			Vector2f newPixelSample = pixelSample;
			Vector3f Lpath = connectLightPath(scene, sampler, lightVertices, cameraVertices, s, t, newPixelSample);

			if (t == 1) m_block->put(newPixelSample, Lpath);
			else L += Lpath;
		}
	}

	return L;
}

}