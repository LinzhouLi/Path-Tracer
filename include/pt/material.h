#pragma once

#include <pt/vector.h>
#include <pt/color.h>

namespace pt {

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

	std::string getName() { return m_name; }

	void setTexture(Bitmap* texture) { m_diffuse_texture = texture; }

	Bitmap* getTexture() { return m_diffuse_texture; }

	Vector3f getBaseColor() { return m_diffuse; }

	Vector3f getBaseColor(const Vector2f& uv);

private:
	std::string m_name;

	Vector3f m_diffuse;
	Vector3f m_specular;
	Vector3f m_transmittance;

	float m_shininess;
	float m_ior;

	Bitmap* m_diffuse_texture = nullptr;
};


float sampleCosineWeightedHemisphere(const Vector2f& u) {

}


class PhongBRDF {
public:
	PhongBRDF(const Vector3f& diffuse, const Vector3f& specular, float shininess) {
		m_Ns = shininess;
		m_Kd = diffuse;
		m_Ks_coef = (m_Ns + 2.0) * INV_TWOPI;
		m_Ks = specular / m_Ks_coef;
		m_p = m_Kd.maxCoeff() / (m_Kd.maxCoeff() + m_Ks.maxCoeff());
	}

	Vector3f f(const Vector3f& wo, const Vector3f& wi) {
		// SameHemisphere
		return m_Kd * INV_PI + m_Ks * m_Ks_coef * std::powf(wo.dot(wi), m_Ns);
	}

	Vector3f sample_f(const Vector3f& wo, float uc, const Vector2f& u) {

	}

	float pdf() {

	}

private:
	Vector3f sampleSpecularLobe(const Vector2f& u) {
		float theta = 2.0f * M_PI * u.y();
		float cos_alpha = std::powf(u.x(), 1.0f / (m_Ns + 1.0f));
		float sin_alpha = std::sqrt(1.0f - cos_alpha * cos_alpha);
		return Vector3f(sin_alpha * std::cos(theta), sin_alpha * std::cosf(theta), cos_alpha);
	}

	Vector3f m_Kd, m_Ks;
	float m_Ns, m_p, m_Ks_coef;
};

}