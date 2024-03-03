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


Vector3f sampleCosineHemisphere(const Vector2f& u) {
	float su0 = std::sqrt(u.x());
	float phi = 2.0f * M_PI * u.y();
	return Vector3f(su0 * std::cos(phi), su0 * std::sin(phi), std::sqrt(1.0f - u.x()));
}

float cosineHemispherePDF(const Vector3f& w) {
	return w.z() * INV_PI;
}

Vector3f reflect(const Vector3f& w, const Vector3f& n) {
	return -w + 2.0f * w.dot(n) * n;
}

class PhongBRDF {
public:
	PhongBRDF(const Vector3f& diffuse, const Vector3f& specular, float shininess) {
		m_Ns = shininess;
		m_Kd = diffuse;
		m_Ks = specular;
	}

	Vector3f f(const Vector3f& wo, const Vector3f& wi, const Vector3f& n) {
		// compute lambert diffuse
		Vector3f diffuse = m_Kd * INV_PI;

		// compute phong specular
		float cosRV = std::max(wi.dot(reflect(wo, n)), 0.0f);
		float normalization = (m_Ns + 2.0f) * INV_TWOPI;
		Vector3f specular = m_Ks * normalization * std::powf(cosRV, m_Ns);

		// return combined result
		return diffuse + specular;
	}

	Vector3f sample_f(const Vector3f& wo, float uc, const Vector2f& u, const Vector2f& n) {
		float sumKd = m_Kd.sum();
		float sumKs = m_Ks.sum();
		float specProb = sumKs / (sumKs + sumKs);
		if (uc < specProb) {
			Vector3f r = reflect(wo, n);
			Vector3f w = sampleSpecularLobe(u);
		}
		else {
			Vector3f w = sampleCosineHemisphere(u);
		}
	}

	float pdf() {

	}

private:
	Vector3f sampleSpecularLobe(const Vector2f& u) {
		float cos_theta = std::powf(u.x(), 1.0f / (m_Ns + 1.0f));
		float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);
		float phi = 2.0f * M_PI * u.y();
		return Vector3f(sin_theta * std::cos(phi), sin_theta * std::cos(phi), cos_theta);
	}

	Vector3f m_Kd, m_Ks;
	float m_Ns;
};

}