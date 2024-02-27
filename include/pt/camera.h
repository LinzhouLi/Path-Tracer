#pragma once

#include <pt/common.h>
#include <pt/transform.h>

namespace pt {

class Camera {
public:
	const float c_near = 0.01f;
	const float c_far = 1000.0f;

	Camera(
		const uint32_t& width = 1280,
		const uint32_t& height = 720,
		const float& fovy = 30,
		const Vector3f& eye = Vector3f(1.0f, 0.0f, 0.0f),
		const Vector3f& lookat = Vector3f(0.0f),
		const Vector3f& up = Vector3f(0.0f, 1.0f, 0.0f)
	);

	Vector2i getScreenSize() { return Vector2i(m_width, m_height); }

	Ray sampleRay(const Vector2f screen_pos);

private:
	uint32_t m_width, m_height;
	float m_fovy;

	Vector3f m_eye;
	Vector3f m_lookat;
	Vector3f m_up;

	Transform m_sample2camera;
	Transform m_camera2world;
};

}