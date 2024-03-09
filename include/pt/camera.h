#pragma once

#include <pt/common.h>
#include <pt/transform.h>

namespace pt {

struct CameraLiSample {
	Vector3f L; // energy loss of camera receiving light
	Vector3f wi; // incident "camear light" direction
	Vector3f p; // world position
	Vector2f pixel; // pixel position
	float pdfDir; // measure in direction
};

class Camera {
public:
	const static float cnear; // 'near' & 'far' are defined in somewhere else
	const static float cfar;
	const static float sample_z;
	static float proj_nume;

	Camera(
		const uint32_t& width = 1280,
		const uint32_t& height = 720,
		const float& fovy = 30,
		const Vector3f& eye = Vector3f(1.0f, 0.0f, 0.0f),
		const Vector3f& lookat = Vector3f(0.0f),
		const Vector3f& up = Vector3f(0.0f, 1.0f, 0.0f)
	);

	Vector2i getScreenSize() const { return Vector2i(m_width, m_height); }

	Ray sampleRay(const Vector2f screen_pos);

	Vector2f project(const Vector3f& p);

	std::pair<Vector3f, Vector2f> Le(const Ray& ray);

	float pdfLe(const Ray& ray);

	CameraLiSample sampleLi(const Intersection& surfIts, const Vector2f& u);

private:
	uint32_t m_width, m_height;
	float m_fovy, m_sample_area;

	Vector3f m_eye;
	Vector3f m_lookat;
	Vector3f m_up;

	Transform m_camera2sample;
	Transform m_sample2camera;
	Transform m_camera2world;
	Transform m_world2camera;
};

}