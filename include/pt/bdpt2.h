#pragma once

#include <pt/integrator.h>
#include <pt/shape.h>
#include <pt/ray.h>

namespace pt {

struct BDPTVertex {
	Vector3f wi;              // in direction
	Vector3f wo;              // out direction
	float rr = 0.0f;          // russian roulette
	Vector3f throughput;      // through put
	Intersection its;         // intersection

	// For further detail, please refer to the paper "Implementing Vertex Connection and Merging"
	// MIS factors
	double vc = 0.0f;
	double vcm = 0.0f;

	// depth of the vertex
	int depth = 0;
};


class BDPTIntegrator2 : public Integrator {
public:
	static constexpr int MaxDepth = 5;

	Vector3f Li(Scene* scene, Sampler* sampler, const Vector2f& pixelSample);

	std::string toString() const {
		return tfm::format("BDPTIntegrator[]");
	}

private:
	// connect camera point
	void connectCamera(const BDPTVertex& light_vertex, Camera* camera, Scene* scene, Sampler* sampler) const;

	// connect light sample
	Vector3f connectLight(const BDPTVertex& eye_vertex, AreaLight* light, Scene* scene, Sampler* sampler) const;

	// connect vertices
	Vector3f connectVertices(const BDPTVertex& light_vertex, const BDPTVertex& eye_vertex, Scene* scene, Sampler* sampler) const;

	inline double MIS(double t) const { return t * t; }
	inline float MIS(float t) const { return t * t; }
};

}