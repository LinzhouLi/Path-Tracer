#pragma once

#include <pt/vector.h>

namespace pt {

class TangentSpace {
public:
	TangentSpace(const Vector3f& tangent, const Vector3f& bitangent, const Vector3f& normal)
		: m_t(tangent), m_b(bitangent), m_n(normal) { }

	TangentSpace(const Vector3f& normal) : m_n(normal) {
		/**
		* Tom Duff, James Burgess, Per Christensen, Christophe Hery, Andrew Kensler, Max Liani, and Ryusuke Villemin, 
		* Building an Orthonormal Basis, Revisited, Journal of Computer Graphics Techniques (JCGT), vol. 6, no. 1, 1-8, 2017
		*/
		float sign = std::copysignf(1.0f, normal.z());
		float a = -1.0f / (sign + normal.z());
		float b = normal.x() * normal.y() * a;
		m_t = Vector3f(1.0f + sign * normal.x() * normal.x() * a, sign * b, -sign * normal.x());
		m_b = Vector3f(b, sign + normal.y() * normal.y() * a, -normal.y());
		if (std::abs(m_n.norm() - 1.0) > 1e-4) cout << "false TangentSpace m_n " << m_n.norm() << endl;
		if (std::abs(m_t.norm() - 1.0) > 1e-4) cout << "false TangentSpace m_t " << m_t.norm() << endl;
		if (std::abs(m_b.norm() - 1.0) > 1e-4) cout << "false TangentSpace m_b " << m_b.norm() << endl;
	}

	Vector3f toLocal(const Vector3f& v) const {
		return Vector3f(m_t.dot(v), m_b.dot(v), m_n.dot(v));
	}

	Vector3f toWorld(const Vector3f& v) const {
		return m_t * v.x() + m_b * v.y() + m_n * v.z();
	}

private:
	Vector3f m_t; // tangent
	Vector3f m_b; // bitangent
	Vector3f m_n; // normal
};

}