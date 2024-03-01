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

}