#pragma once

#include <pt/vector.h>

namespace pt {

class TriangleSample;

class AreaLight {
public:
	AreaLight(Triangle* shape, const Vector3f& lemit);

	// surface normal, ray direction
	Vector3f L(const Vector3f& n, const Vector3f& w) const;

	Vector3f power() const { return m_area * m_lemit * M_PI; }

	TriangleSample sampleLi(const Vector2f& u) const;

private:
	Triangle* m_shape;
	Vector3f m_lemit;
	float m_area;

};

}