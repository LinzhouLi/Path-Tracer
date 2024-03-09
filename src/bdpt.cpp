#include <pt/bdpt.h>
#include <pt/scene.h>
#include <pt/sampler.h>
#include <pt/ray.h>
#include <pt/material.h>
#include <pt/light.h>
#include <pt/camera.h>
#include <pt/block.h>

namespace pt {

	bool isValid(const Vector3f& v) {
		for (int i = 0; i < 3; ++i) {
			float value = v.coeff(i);
			if (value < 0 || !std::isfinite(value))
				return false;
		}
		return true;
	}

	bool isValid(float v) {
		if (v < 0 || !std::isfinite(v)) return false;
		return true;
	}


float correctShadingNormal(const Intersection& its, const Vector3f& wo, const Vector3f& wi, TransportMode mode) {
	if (mode == TransportMode::Radiance) {
		return 1.0f;
	}
	else {
		float num = absDot(its.n, wo) * absDot(its.ng, wi); // cos(wo, ns) * cos(wi, ng)
		float denom = absDot(its.ng, wo) * absDot(its.n, wi); // cos(wo, ng) * cos(wi, ns)
		if (denom == 0.0f) return 0.0f;
		else return num / denom;
	}
}

float G(const Vertex& a, const Vertex& b) {
	Vector3f ab = b.its.p - a.its.p;
	float dist = ab.norm();
	if (dist == 0.0f) return 0.0f;
	ab /= dist;
	return absDot(a.its.n, ab) * absDot(b.its.n, -ab) / (dist * dist);
}


inline Vertex Vertex::createFromLight(AreaLight* light, const Vector3f& position, const Vector3f& normal, const Vector3f& Le, float pdf) {
	Vertex v;
	v.type = VertexType::Light;
	v.its.p = position;
	v.its.n = v.its.ng = normal;
	v.pdfAreaFwd = pdf;
	v.light = light;
	v.beta = Le;
	return v;
}

inline Vertex Vertex::createFromCamera(Camera* camera, const Vector3f& position, const Vector3f& beta) {
	Vertex v;
	v.type = VertexType::Camera;
	v.its.p = position;
	v.camera = camera;
	v.beta = beta;
	return v;
}

inline Vertex Vertex::createFromSurface(const Intersection& its, const Vector3f& beta) {
	Vertex v;
	v.type = VertexType::Surface;
	v.its = its;
	v.beta = beta;
	return v;
}

float Vertex::convertPdfDensity(float pdf, const Vertex& nextVertex) const {
	Vector3f w = nextVertex.its.p - this->its.p;
	float dist = w.norm();
	if (dist == 0.0f) return 0.0f;
	if (nextVertex.type != VertexType::Camera) {
		w /= dist;
		pdf *= absDot(nextVertex.its.n, w); // why ng?
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

float Vertex::pdfLightOrigin(Scene* scene, const Vertex* v) const {
	Vector3f w = v->its.p - this->its.p;
	float dist = w.norm();
	if (dist == 0.0f) return 0.0f;
	w /= dist;

	const AreaLight* lit = type == VertexType::Light ? light : its.getLight();
	float pdfSelect = scene->getLightSelector()->pdf(lit);
	float pdfArea = lit->pdfArea();
	return pdfArea * pdfSelect;
}

float Vertex::pdfLight(const Vertex* v) const {
	Vector3f w = v->its.p - this->its.p;
	float dist = w.norm();
	if (dist == 0.0f) return 0.0f;
	w /= dist;

	const AreaLight* lit = type == VertexType::Light ? light : its.getLight();
	float pdfDir = lit->pdfDir(w, its.n);
	return absDot(v->its.n, w) * pdfDir / (dist * dist);
}

float Vertex::pdf(const Vertex* prev, const Vertex* next) const {
	if (type == VertexType::Light) return pdfLight(next);

	Vector3f wn = next->its.p - this->its.p;
	float dist = wn.norm();
	if (dist == 0.0f) return 0.0f;
	wn /= dist;

	Vector3f wp;
	if (prev) {
		wp = prev->its.p - this->its.p;
		float dist = wp.norm();
		if (dist == 0.0f) return 0.0f;
		wp /= dist;
	}

	float pdf = 0.0f;
	if (type == VertexType::Camera) pdf = this->camera->pdfLe(Ray(its.p, wn, 0.0f));
	else if (type == VertexType::Surface) pdf = its.pdfBRDF(wn, wp);
	return convertPdfDensity(pdf, *next);
}


int BDPTIntegrator::randomWalk(
	Scene* scene, Sampler* sampler, Vertex* path, Ray ray,
	Vector3f beta, float pdf, int maxDepth, TransportMode mode
) const {
	if (maxDepth == 0) return 0;

	int bounce = 0;
	float pdfDirFwd = pdf, pdfDirRev = 0;
	while(1) {
		Intersection its;
		bool hit = scene->rayIntersect(ray, its);
		if (!hit) break;

		Vertex& vertex = path[bounce];
		Vertex& preVertex = path[bounce - 1];

		// create surface vertex
		vertex = Vertex::createFromSurface(its, beta);
		vertex.pdfAreaFwd = preVertex.convertPdfDensity(pdfDirFwd, vertex);
		if (++bounce >= maxDepth) break;

		// sample BRDF
		Vector3f wo = -ray.dir;
		BRDFSample bs = its.sampleBRDF(wo, sampler->sample1D(), sampler->sample2D());
		if (bs.f.squaredNorm() == 0.0f || bs.pdf == 0.0f) break; // no enerage

		// accumulate beta (throughput)
		Vector3f throughput = bs.f * absDot(its.n, bs.wi) / bs.pdf;
		beta = beta.cwiseProduct(throughput);
		beta *= correctShadingNormal(its, wo, bs.wi, mode);

		// record pdf
		pdfDirFwd = bs.pdf;
		pdfDirRev = its.pdfBRDF(bs.wi, wo); // reverse wi and wo

		// compute reverse area density at prev vertex
		preVertex.pdfAreaRev = vertex.convertPdfDensity(pdfDirRev, preVertex);

		// new ray
		ray = its.genRay(bs.wi);
	}
	return bounce;
}

int BDPTIntegrator::generateCameraSubpath(Scene* scene, Sampler* sampler, Vertex* path, const Vector2f& pixelSample, int maxDepth) const {
	if (maxDepth == 0) return 0;

	Vector3f beta(1.0f);
	Ray ray = scene->getCamera()->sampleRay(pixelSample);

	// create first vertex
	path[0] = Vertex::createFromCamera(scene->getCamera(), ray.org, beta);
	float pdfDir = scene->getCamera()->pdfLe(ray);

	// sample other vertices
	int numVertices = randomWalk(
		scene, sampler, path + 1, ray,
		beta, pdfDir, maxDepth - 1, TransportMode::Radiance
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
	Vector3f beta = ls.L * absDot(ls.n, ls.ray.dir) / (ls.pdfArea * ls.pdfDir * selectPdf);
	path[0] = Vertex::createFromLight(light, ls.ray.org, ls.n, ls.L, ls.pdfArea * selectPdf);

	// sample other vertices
	int numVertices = randomWalk(
		scene, sampler, path + 1, ls.ray, 
		beta, ls.pdfDir, maxDepth - 1, TransportMode::Importance
	);
	return numVertices + 1;
}

float BDPTIntegrator::computeMISWeight(
	Scene* scene, Vertex& sampled,
	Vertex* lightVertices, Vertex* cameraVertices,
	int s, int t
) const {
	if (s + t == 2) return 1.0f;
	float sumRi = 0.0f;

	// deals with Dirac delta functions
	auto remap0 = [](float f) -> float { return f != 0.0f ? f : 1.0f; };

	// Look up connection vertices and their predecessors
	Vertex* qs = s > 0 ? &lightVertices[s - 1] : nullptr;
	Vertex* pt = t > 0 ? &cameraVertices[t - 1] : nullptr;
	Vertex* qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr;
	Vertex* ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

	// Update sampled vertex for $s=1$ or $t=1$ strategy
	ScopedAssignment<Vertex> a1;
	if (s == 1) a1 = { qs, sampled };
	else if (t == 1) a1 = { pt, sampled };

	// Update reverse density of vertex $\pt{}_{t-1}$
	ScopedAssignment<float> a4;
	if (pt) a4 = { &pt->pdfAreaRev, s > 0 ? qs->pdf(qsMinus, pt) : pt->pdfLightOrigin(scene, ptMinus) };

	// Update reverse density of vertex $\pt{}_{t-2}$
	ScopedAssignment<float> a5;
	if (ptMinus) a5 = { &ptMinus->pdfAreaRev, s > 0 ? pt->pdf(qs, ptMinus) : pt->pdfLight(ptMinus) };

	// Update reverse density of vertices $\pq{}_{s-1}$ and $\pq{}_{s-2}$
	ScopedAssignment<float> a6;
	if (qs) a6 = { &qs->pdfAreaRev, pt->pdf(ptMinus, qs) };
	ScopedAssignment<float> a7;
	if (qsMinus) a7 = { &qsMinus->pdfAreaRev, qs->pdf(pt, qsMinus) };

	// Consider hypothetical connection strategies along the camera subpath
	float ri = 1;
	for (int i = t - 1; i > 0; --i) {
		ri *= remap0(cameraVertices[i].pdfAreaRev) / remap0(cameraVertices[i].pdfAreaFwd);
		sumRi += ri;
	}

	// Consider hypothetical connection strategies along the light subpath
	ri = 1;
	for (int i = s - 1; i >= 0; --i) {
		ri *= remap0(lightVertices[i].pdfAreaRev) / remap0(lightVertices[i].pdfAreaFwd);
		sumRi += ri;
	}
	return 1 / (1 + sumRi);
}


Vector3f BDPTIntegrator::Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample) {
	Vector3f L(0.0);

	// generate subpaths
	Vertex cameraVertices[MaxDepth + 2], lightVertices[MaxDepth + 1];
	int numCameraVs = generateCameraSubpath(scene, sampler, cameraVertices, pixelSample, MaxDepth + 2);
	int numLightVs = generateLightSubpath(scene, sampler, lightVertices, MaxDepth + 1);

	// connect subpaths and compute contribution
	for (int t = 1; t <= numCameraVs; t++) {
		for (int s = 0; s <= numLightVs; s++) {
			int depth = t + s - 2;
			if ((s == 1 && t == 1) || depth < 0 || depth > MaxDepth) continue;

			if (s == 0) { // use full camera path, has energy only when the last vertex emits light
				L += connectCameraPathWithLight(scene, lightVertices, cameraVertices, t);
			}
			else if (t == 1) { // resample a point on a camera and connect it to the light subpath.
				//auto ret = connectPathSampleCamera(scene, sampler, lightVertices, cameraVertices, s);
				//if (ret.has_value()) {
				//	auto& [Lpath, pixel] = ret.value();
				//	m_block->put(pixel, Lpath); // pbrt: add splat???
				//}
			}
			else if (s == 1) {// resample a point on a light and connect it to the camera subpath.
				L += connectPathSampleLight(scene, sampler, lightVertices, cameraVertices, t);
			}
			else { // general cases
				L += connectPath(scene, lightVertices, cameraVertices, s, t);
			}

		}
	}

	return L;
}

Vector3f BDPTIntegrator::connectCameraPathWithLight(
	Scene* scene, Vertex* lightVertices, Vertex* cameraVertices, int t
) const {
	Vector3f L(0.0);
	Vertex sampledVertex;

	const Vertex& vt = cameraVertices[t - 1];
	const Vertex& vt_prev = cameraVertices[t - 2];
	if (vt.its.getLight()) {
		Vector3f w = vt_prev.its.p - vt.its.p;
		float dist = w.norm();
		if (dist == 0.0f) return Vector3f(0.0f);
		w /= dist;
		L = vt.beta.cwiseProduct(vt.its.Le(w));
	}

	float weight = 1.0f;
	if (L.squaredNorm() != 0.0f)
		weight = computeMISWeight(scene, sampledVertex, lightVertices, cameraVertices, 0, t);
	return L * weight;
}

Vector3f BDPTIntegrator::connectPathSampleLight(
	Scene* scene, Sampler* sampler,
	Vertex* lightVertices, Vertex* cameraVertices, int t
) const {
	Vector3f L(0.0);
	Vertex sampledVertex;

	const Vertex& vt = cameraVertices[t - 1];
	const Vertex& vt_prev = cameraVertices[t - 2];

	// select light
	AreaLight* light = scene->getLightSelector()->select(sampler->sample1D());
	float selectPdf = scene->getLightSelector()->pdf(light);

	// sample incident ray on the light shape
	LightLiSample ls = light->sampleLi(vt.its, sampler->sample2D());
	if (
		ls.pdfDir != 0.0f && ls.L.squaredNorm() != 0.0f &&
		scene->unocculded(vt.its.p, ls.p, vt.its.n, ls.n) // visibility test
		) {
		float pdfLight = ls.pdfDir * selectPdf;
		sampledVertex = Vertex::createFromLight(light, ls.p, ls.n, ls.L / pdfLight, 0.0f);
		L = vt.beta
			.cwiseProduct(vt.BRDF(vt_prev, sampledVertex, TransportMode::Radiance))
			.cwiseProduct(sampledVertex.beta) * absDot(vt.its.n, ls.wi);
	}

	float weight = 1.0f;
	if (L.squaredNorm() != 0.0f)
		weight = computeMISWeight(scene, sampledVertex, lightVertices, cameraVertices, 1, t);
	return L * weight;
}

std::optional<std::pair<Vector3f, Vector2f>> BDPTIntegrator::connectPathSampleCamera(
	Scene* scene, Sampler* sampler,
	Vertex* lightVertices, Vertex* cameraVertices, int s
) const {
	Vector3f L(0.0);
	Vertex sampledVertex;

	const Vertex& vs = lightVertices[s - 1];
	const Vertex& vs_prev = lightVertices[s - 2];

	Camera* camera = scene->getCamera(); // only one camera, no need to select

	CameraLiSample cs = camera->sampleLi(vs.its, sampler->sample2D());
	auto pixel = camera->project(vs.its.p);
	if (!pixel.has_value() || !scene->unocculded(vs.its.p, cs.p, vs.its.n)) return std::nullopt;

	if (cs.pdfDir != 0.0f && cs.L.squaredNorm() != 0.0f) {
		sampledVertex = Vertex::createFromCamera(camera, cs.p, cs.L / cs.pdfDir);
		L = vs.beta
			.cwiseProduct(vs.BRDF(vs_prev, sampledVertex, TransportMode::Importance))
			.cwiseProduct(sampledVertex.beta) * absDot(vs.its.n, cs.wi);
	}

	float weight = 1.0f;
	if (L.squaredNorm() != 0.0f)
		weight = computeMISWeight(scene, sampledVertex, lightVertices, cameraVertices, s, 1);
	return std::make_pair(L, pixel.value());
}

Vector3f BDPTIntegrator::connectPath(
	Scene* scene, Vertex* lightVertices, Vertex* cameraVertices,
	int s, int t
) const {
	Vector3f L(0.0);
	Vertex sampledVertex;

	const Vertex& vs = lightVertices[s - 1];
	const Vertex& vt = cameraVertices[t - 1];
	if (scene->unocculded(vs.its.p, vt.its.p, vs.its.n, vt.its.n)) {
		const Vertex& vs_prev = lightVertices[s - 2];
		const Vertex& vt_prev = cameraVertices[t - 2];
		L = vs.beta
			.cwiseProduct(vs.BRDF(vs_prev, vt, TransportMode::Importance))
			.cwiseProduct(vt.BRDF(vt_prev, vs, TransportMode::Radiance))
			.cwiseProduct(vt.beta);
		if (L.squaredNorm() != 0.0f) L *= G(vs, vt);
	}

	float weight = 1.0f;
	if (L.squaredNorm() != 0.0f)
		weight = computeMISWeight(scene, sampledVertex, lightVertices, cameraVertices, s, t);
	return L * weight;
}

}