#include <pt/light.h>
#include <pt/shape.h>

namespace pt {

AreaLight::AreaLight(Triangle* shape, const Vector3f& lemit) : m_shape(shape), m_lemit(lemit) {
	m_area = shape->surfaceArea();
}

Vector3f AreaLight::L(const Vector3f& n, const Vector3f& w) const {
	return n.dot(w) > 0.0 ? m_lemit : Vector3f(0.0);
}

TriangleSample AreaLight::sampleLi(const Vector2f& u) const {
	return m_shape->sample(u);
}

}