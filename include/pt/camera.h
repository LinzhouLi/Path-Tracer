#pragma once

#include <pt/common.h>

namespace pt {

class Camera {
public:
	Camera(
		const uint32_t& width = 1024u,
		const uint32_t& height = 1024u,
		const float& fovy = 45,
		const Vector3f& eye = Vector3f(1.0f, 0.0f, 0.0f),
		const Vector3f& lookat = Vector3f(0.0f),
		const Vector3f& up = Vector3f(0.0f, 1.0f, 0.0f)
	) : m_width(width), m_height(height), m_fovy(fovy), m_eye(eye), m_lookat(lookat), m_up(up) { }

private:
	uint32_t m_width, m_height;
	float m_fovy;

	Vector3f m_eye;
	Vector3f m_lookat;
	Vector3f m_up;
};

}