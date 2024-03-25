#include <pt/material.h>
#include <pt/color.h>
#include <pt/bitmap.h>
#include <pt/shape.h>
#include <pt/tangent.h>

namespace pt {

Vector3f Material::getBaseColor(const Vector2f& uv) const {
	if (m_diffuse_texture) {
		Color3f c = m_diffuse_texture->sample(uv);
		return Vector3f(c.x(), c.y(), c.z());
	}
	else
		return getBaseColor();
}

Vector3f Material::BRDF(const Vector3f& wo, const Vector3f& wi, const Intersection& its) const {
	// not on the same hemisphere
	//float cosTheta = wi.dot(its.n);
	//if (cosTheta < 0.0f) return Vector3f(0.0f);

	// compute lambert diffuse
	Vector3f diffuse = getBaseColor(its.uv) * INV_PI;

	// compute phong specular
	Vector3f r = reflect(wo, its.n);
	float cosRV = std::max(wi.dot(r), 0.0f);
	float normalization = (m_shininess + 2.0f) * INV_TWOPI;
	Vector3f specular = m_specular * normalization * std::powf(cosRV, m_shininess);

	// return combined result
	return diffuse + specular;
}

BRDFSample Material::sampleBRDF(const Vector3f& wo, float uc, const Vector2f& u, const Intersection& its) const {
	/**
	* Lafortune, Eric P. and Yves D. Willems. “Using the modified Phong reflectance model for physically based rendering.” (1994).
	*/
	Vector3f diffuse = getBaseColor(its.uv); // sample diffuse color

	float sumKd = diffuse.sum();
	float sumKs = m_specular.sum();
	float sumKdKs = sumKd + sumKs;
	if (sumKdKs == 0.0f) return BRDFSample(); // black body
	float specProb = sumKs / sumKdKs;

	Vector3f wi;
	Vector3f r = reflect(wo, its.n);
	if (uc < specProb) { // sample specular
		TangentSpace ts(r);
		Vector3f w = samplePhongSpecularLobe(u, m_shininess);
		wi = ts.toWorld(w);
	}
	else { // sample diffuse
		Vector3f w = sampleCosineHemisphere(u);
		wi = its.ts.toWorld(w);
	}
	wi.normalize();

	// not on the same hemisphere
	float cosTheta = wi.dot(its.n);
	if (cosTheta < 0.0f) return BRDFSample();

	// specular pdf
	float cosRV = std::max(wi.dot(r), 0.0f);
	float powRV = std::powf(cosRV, m_shininess);
	float pdf_spec = (m_shininess + 1.0f) * INV_TWOPI * powRV;

	// diffuse pdf
	float pdf_diff = cosTheta * INV_PI;

	// BRDF value
	Vector3f f = diffuse * INV_PI + m_specular * (m_shininess + 2.0f) * INV_TWOPI * powRV;

	float pdf = mix(pdf_diff, pdf_spec, specProb);
	return BRDFSample(wi, pdf, f);
}

float Material::pdf(const Vector3f& wo, const Vector3f& wi, const Intersection& its) const {
	Vector3f diffuse = getBaseColor(its.uv);

	float sumKd = diffuse.sum();
	float sumKs = m_specular.sum();
	float sumKdKs = sumKd + sumKs;
	if (sumKdKs == 0.0f) return 0.0f; // black body
	float specProb = sumKs / sumKdKs;
	Vector3f r = reflect(wo, its.n);

	// specular pdf
	float cosRV = std::max(wi.dot(r), 0.0f);
	float pdf_spec = (m_shininess + 1.0f) * INV_TWOPI * std::powf(cosRV, m_shininess);

	// diffuse pdf
	float cosTheta = absDot(wi, its.n); // may be incorrect
	float pdf_diff = cosTheta * INV_PI;

	return mix(pdf_diff, pdf_spec, specProb);
}

std::string Material::toString() const {
	return tfm::format(
		"Material[\n"
		"  name = %s,\n"
		"  diffuse = %s,\n"
		"  specular = %s,\n"
		"  shininess = %f,\n"
		"  texture = %s\n"
		"]",
		m_name,
		m_diffuse.toString(),
		m_specular.toString(),
		m_shininess,
		m_diffuse_texture ? indent(m_diffuse_texture->toString()) : "null"
	);
}

}
