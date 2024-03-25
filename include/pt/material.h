#pragma once

#include <pt/vector.h>
#include <pt/color.h>

namespace pt {

struct BRDFSample {
	BRDFSample(const Vector3f& wi_ = Vector3f(), float pdf_ = 0.0f, Vector3f f_ = Vector3f()) : wi(wi_), pdf(pdf_), f(f_) { }

	Vector3f wi;
	float pdf;
	Vector3f f;
};

class Material {
public:
	Material(
		const std::string& name = "",
		const Vector3f& diffuse = Vector3f(0.5f),
		const Vector3f& specular = Vector3f(0.0f),
		const Vector3f& transmittance = Vector3f(1.0f),
		const float shiness = 1.0f,
		const float ior = 1.0f
	) : m_name(name), m_diffuse(diffuse), m_specular(specular), m_transmittance(transmittance), m_shininess(shiness), m_ior(ior) { }

	~Material() {
		if (m_diffuse_texture != nullptr)
			delete m_diffuse_texture;
	}

	std::string getName() const { return m_name; }

	void setTexture(Bitmap* texture) { m_diffuse_texture = texture; }

	Bitmap* getTexture() const { return m_diffuse_texture; }

	Vector3f getBaseColor() const { return m_diffuse; }

	Vector3f getBaseColor(const Vector2f& uv) const;

	Vector3f BRDF(const Vector3f& wo, const Vector3f& wi, const Intersection& its) const;

	BRDFSample sampleBRDF(const Vector3f& wo, float uc, const Vector2f& u, const Intersection& its) const;

	float pdf(const Vector3f& wo, const Vector3f& wi, const Intersection& its) const;

	std::string toString() const;

private:
	std::string m_name;

	Vector3f m_diffuse;
	Vector3f m_specular;
	Vector3f m_transmittance;

	float m_shininess;
	float m_ior;

	Bitmap* m_diffuse_texture = nullptr;
};

}