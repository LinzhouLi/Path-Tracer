#include <pt/camera.h>
#include <pt/ray.h>
#include <pt/shape.h>

namespace pt {

const float Camera::cnear = 0.01f;
const float Camera::cfar = 10000.0f;
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
		0.0, 0.0, far / (far - near), -(far * near) / (far - near), // z = zfar, depth = 1.0; z = znear, depth = 0.0
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

	// raster area (image plane bounds at z = 1.0)
	Vector3f pMin = m_sample2camera.apply(Vector3f(0.0f, 0.0f, Camera::sample_z), Transform::Type::Scaler);
	Vector3f pMax = m_sample2camera.apply(Vector3f(float(width), float(height), Camera::sample_z), Transform::Type::Scaler);
	pMin /= pMin.z();
	pMax /= pMax.z();
	m_sample_area = std::abs((pMax.x() - pMin.x()) * (pMax.y() - pMin.y()));

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
	Vector3f p_ndc = m_camera2sample.apply(p_cam, Transform::Type::Scaler);
	if (
		p_ndc.z() < 0.0f || p_ndc.z() > 1.0f ||
		p_ndc.x() < 0.0f || p_ndc.x() > float(m_width) ||
		p_ndc.y() < 0.0f || p_ndc.y() > float(m_height)
	) return std::nullopt;
	return Vector2f(p_ndc.x(), p_ndc.y());
}

// equation (16.4) from PBRT-v3
// https://www.pbr-book.org/3ed-2018/Light_Transport_III_Bidirectional_Methods/The_Path-Space_Measurement_Equation#eq:importance-area
Vector3f Camera::Le(const Vector3f& w) { // 'W_e' in the equation 
	Vector3f camForward = (m_lookat - m_eye).normalized();
	float cosTheta = camForward.dot(w);
	if (cosTheta <= 0.0f) return Vector3f(0.0f);

	// return importance for point on image plane
	float cosTheta2 = cosTheta * cosTheta;
	return Vector3f(1.0f / (m_sample_area * cosTheta2 * cosTheta2));
}

float Camera::pdfLe(const Ray& ray) {
	Vector3f camForward = (m_lookat - m_eye).normalized();
	float cosTheta = camForward.dot(ray.dir);
	if (cosTheta <= 0.0f) return 0.0f;

	//float pdfArea = 1.0f;
	float pdfDir = 1.0f / (m_sample_area * cosTheta * cosTheta * cosTheta);
	return pdfDir;
}

CameraLiSample Camera::sampleLi(const Intersection& surfIts, const Vector2f& u) {
	Vector3f wi = m_eye - surfIts.p;
	Vector3f camForward = (m_lookat - m_eye).normalized();
	float dist = wi.norm();
	wi /= dist;

	// compute PDF of importance arriving at surfIts
	float pdfDir = dist * dist / absDot(camForward, wi);

	// compute importance
	Vector3f L = Le(-wi);
	return CameraLiSample { L, wi, m_eye, pdfDir };
}


std::string Camera::toString() const {
	return tfm::format(
		"PerspectiveCamera[\n"
		"  width = %i, height = %i,\n"
		"  near = %f, far = %f,\n"
		"  eye = %s,\n"
		"  lookat = %s,\n"
		"  fov = %f\n"
		"]",
		m_width, m_height,
		Camera::cnear, Camera::cfar,
		m_eye.toString(),
		m_lookat.toString(),
		m_fovy
	);
}

};