#pragma once

#include <pt/vector.h>

namespace pt {

class Camera {
public:
	Camera(
		const uint32_t& width = 1280,
		const uint32_t& height = 720,
		const float& fovy = 30,
		const Vector3f& eye = Vector3f(1.0f, 0.0f, 0.0f),
		const Vector3f& lookat = Vector3f(0.0f),
		const Vector3f& up = Vector3f(0.0f, 1.0f, 0.0f)
	);

private:
	uint32_t m_width, m_height;
	float m_fovy;

	Vector3f m_eye;
	Vector3f m_lookat;
	Vector3f m_up;
};

}