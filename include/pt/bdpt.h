#pragma once

#include <pt/integrator.h>
#include <pt/shape.h>
#include <pt/ray.h>

namespace pt {

// radiance transport: from camera, importance transport: from light
enum class TransportMode { Radiance, Importance };

enum class VertexType { Camera, Light, Surface };

struct Vertex {
public:

	static inline Vertex createFromLight(const AreaLight* light, const Vector3f& position, const Vector3f& normal, const Vector3f& Le, float pdf);

	static inline Vertex createFromCamera(const Camera* camera, const Vector3f& position, const Vector3f& dir, const Vector3f& beta);

	static inline Vertex createFromSurface(const Intersection& its, const Vector3f& beta);

	float convertPdfDensity(float pdf, const Vertex& nextVertex) const;

	Vector3f Le(const Vertex& v) const;

	Vector3f BRDF(const Vertex& preVertex, const Vertex& nextVertex, TransportMode mode) const;

	union {
		const AreaLight* light;
		const Camera* camera;
	};

	VertexType type;
	Vector3f beta;
	float pdfAreaFwd = 0.0; // area measure
	float pdfAreaRev = 0.0; // area measure
	Intersection its;
};


class BDPTIntegrator : public Integrator {
public:
	static constexpr int MaxDepth = 5;

	void preprocess(Scene* scene) {
		cout << "BDPT PathIntegrator!" << endl;
	}

	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

private:
	int generateCameraSubpath(Scene* scene, Sampler* sampler, Vertex* path, const Vector2f& pixelSample, int maxDepth) const;
	int generateLightSubpath(Scene* scene, Sampler* sampler, Vertex* path, int maxDepth) const;

	int randomWalk(
		Scene* scene, Sampler* sampler, Vertex* path, Ray ray,
		Vector3f beta, float pdf, int maxDepth, TransportMode mode
	) const;

	Vector3f connectLightPath(
		Scene* scene, Sampler* sampler, 
		Vertex* lightVertices, Vertex* cameraVertices,
		int s, int t, Vector2f& pixelSample
	) const;

	float computeMISWeight(
		Scene* scene,
		Vertex* lightVertices, Vertex* cameraVertices,
		int s, int t
	) const;
};

}