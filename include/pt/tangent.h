#pragma once

#include <pt/vector.h>

namespace pt {

class TangentSpace {
public:
	TangentSpace(const Vector3f& tangent, const Vector3f& bitangent, const Vector3f& normal)
		: m_t(tangent), m_b(bitangent), m_n(normal) { }

	TangentSpace(const Vector3f& normal) : m_n(normal) {
		if (std::abs(m_n.x()) > std::abs(m_n.y())) {
			m_b = Vector3f(m_n.z(), 0, -m_n.x()) / sqrt(m_n.x() * m_n.x() + m_n.z() * m_n.z());
		} else {
			m_b = Vector3f(0, m_n.z(), -m_n.y()) / sqrt(m_n.y() * m_n.y() + m_n.z() * m_n.z());
		}
		m_t = m_b.cross(m_n);
	}

	Vector3f toLocal(const Vector3f& v) const {
		return Vector3f(m_t.dot(v), m_b.dot(v), m_n.dot(v));
	}

	Vector3f toWorld(const Vector3f& v) const {
		return m_t * v.x() + m_b * v.y() + m_n * v.z();
	}

private:
	Vector3f m_t;
	Vector3f m_b;
	Vector3f m_n;
};

}