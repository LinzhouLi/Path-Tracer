#include <pt/camera.h>

namespace pt {

//Eigen::Matrix4f computeCameraToWorldMatrix(const Vector3f& eye, const Vector3f& lookat, const Vector3f& up) {
//	Vector3f forward = (lookat - eye).normalized();
//	Vector3f right = forward.cross(up).normalized();
//	Vector3f newUp = right.cross(forward);
//
//	Eigen::Matrix4f cameraToWorld;
//	cameraToWorld << 
//		right.x(), right.y(), right.z(), -right.dot(eye),
//		newUp.x(), newUp.y(), newUp.z(), -newUp.dot(eye),
//		-forward.x(), -forward.y(), -forward.z(), forward.dot(eye),
//		0, 0, 0, 1;
//
//	return cameraToWorld;
//}

Camera::Camera(
	const uint32_t& width,
	const uint32_t& height,
	const float& fovy,
	const Vector3f& eye,
	const Vector3f& lookat,
	const Vector3f& up
) : m_width(width), m_height(height), m_fovy(fovy), m_eye(eye), m_lookat(lookat), m_up(up) {
	//m_camera2world = computeCameraToWorldMatrix(eye, lookat, up);
}

};