#include <pt/camera.h>

namespace pt {

const float Camera::cnear = 0.01f;
const float Camera::cfar = 1000.0f;
const float Camera::sample_z = 0.5f;
float Camera::proj_nume = 1.0f;

inline float deg2rad(float value) { 
	return value * (M_PI / 180.0f); 
}

inline float fov2focal(float fov, float pixels) {
	return pixels / (2.0 * std::tan(deg2rad(0.5 * fov)));
}

Eigen::Matrix4f computeCameraToWorldMatrix(const Vector3f& eye, const Vector3f& lookat, const Vector3f& up) {
	Vector3f forward = (lookat - eye).normalized();
	Vector3f right = forward.cross(up).normalized();
	Vector3f newUp = right.cross(forward);

	Eigen::Matrix4f cameraToWorld;
	cameraToWorld << 
		-right, newUp, forward, eye,
		0, 0, 0, 1;

	return cameraToWorld;
}

Eigen::Matrix4f computeProjectionMatrix(float aspect, float fovy, float near, float far) {
	float fovx = fovy * aspect;
	float cotx = 1.0f / std::tan(deg2rad(0.5 * fovx));
	float coty = 1.0f / std::tan(deg2rad(0.5 * fovy));
	Eigen::Matrix4f projection;
	projection <<
		cotx, 0.0, 0.0, 0.0,
		0.0, coty, 0.0, 0.0,
		0.0, 0.0, far / (far - near), -(far * near) / (far - near),
		0.0, 0.0, 1.0, 0.0;
	return projection;
}

Camera::Camera(
	const uint32_t& width,
	const uint32_t& height,
	const float& fovy,
	const Vector3f& eye,
	const Vector3f& lookat,
	const Vector3f& up
) : m_width(width), m_height(height), m_fovy(fovy), m_eye(eye), m_lookat(lookat), m_up(up) {

	float aspect = float(width) / float(height); // caution: integer divide!
	Eigen::Matrix4f camera2world_mat = computeCameraToWorldMatrix(eye, lookat, up);
	Eigen::Matrix4f projection_mat = computeProjectionMatrix(aspect, fovy, Camera::cnear, Camera::cfar);

	Eigen::Affine3f ndc2pixel_trans = Eigen::Affine3f::Identity();
	ndc2pixel_trans.translate(Eigen::Vector3f(0.5 * float(width), 0.5 * float(height), 0.0));
	ndc2pixel_trans.scale(Eigen::Vector3f(-0.5 * float(width), -0.5 * float(height), 1.0));
	Eigen::Matrix4f ndc2pixel_mat = ndc2pixel_trans.matrix();

	m_camera2sample.setMatrix(ndc2pixel_mat * projection_mat);
	m_sample2camera.setMatrix((ndc2pixel_mat * projection_mat).inverse());
	m_camera2world.setMatrix(camera2world_mat);
	m_world2camera.setMatrix(camera2world_mat.inverse());

	// for projecton correction
	Vector3f tmp_sample(0.5f * width, 0.5f * height, Camera::sample_z);
	tmp_sample = m_sample2camera.apply(tmp_sample, Transform::Type::Scaler);
	Camera::proj_nume = 1.0 / tmp_sample.z();
}

Ray Camera::sampleRay(const Vector2f screen_pos) {
	Vector3f d(screen_pos.x(), screen_pos.y(), Camera::sample_z);
	d = m_sample2camera.apply(d, Transform::Type::Scaler);
	d.normalize();
	float proj = Camera::proj_nume / d.z(); // project to a plane
	d = m_camera2world.apply(d, Transform::Type::Vector);
	return Ray(m_eye, d, Camera::cnear * proj, Camera::cfar * proj);
}

std::optional<Vector2f> Camera::project(const Vector3f& p) {
	Vector3f p_cam = m_world2camera.apply(p, Transform::Type::Scaler);
	if (p_cam.z() < Camera::cnear || p_cam.z() > Camera::cfar)
		return std::nullopt;
	Vector3f p_ndc = m_camera2sample.apply(p_cam, Transform::Type::Scaler);
	if (p_ndc.x() < 0 || p_ndc.x() > m_width || p_ndc.y() < 0 || p_ndc.y() > m_height)
		return std::nullopt;
	return Vector2f(p_ndc.x(), p_ndc.y());
}

};