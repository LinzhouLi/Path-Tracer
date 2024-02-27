#pragma once

#include <pt/common.h>
#include <pt/vector.h>

namespace pt {

class AABB {
public:
	// Initialize an empty AABB
	AABB() {
		m_min = Vector3f(std::numeric_limits<float>::infinity());
		m_max = Vector3f(std::numeric_limits<float>::lowest());
	}

	AABB(const AABB& b) : m_min(b.m_min), m_max(b.m_max) { }

	// Initialize AABB with 1 vector
	AABB(const Vector3f& v) : m_min(v), m_max(v) { }

	// Initialize AABB with 2 vector
	AABB(const Vector3f& v1, const Vector3f& v2) : m_min(v1), m_max(v1) {
		*this += v2;
	}

	// Initialize AABB with 3 vector
	AABB(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3) : m_min(v1), m_max(v1) {
		*this += v2;
		*this += v3;
	}

	// Add a vector into this AABB
	inline AABB& operator += (const Vector3f& p) {
		m_min = m_min.cwiseMin(p);
		m_max = m_max.cwiseMax(p);
		return *this;
	}

	// Add another AABB into this AABB
	inline AABB& operator += (const AABB& b) {
		m_min = m_min.cwiseMin(b.m_min);
		m_max = m_max.cwiseMax(b.m_max);
		return *this;
	}

	// Add a vector into this AABB
	inline AABB operator + (const Vector3f& p) const {
		AABB r(*this);
		return r += p;
	}

	// Add another AABB into this AABB
	inline AABB operator + (const AABB& b) const {
		AABB r(*this);
		return r += b;
	}

	// Check if AABB is empty
	inline bool empty() const { m_max.x() < m_min.x(); }

	// Center of AABB
	inline Vector3f center() const { return (m_max + m_min) * 0.5; }

	// Axis length of AABB
	inline float width() const { return m_max.x() - m_min.x(); }
	inline float height() const { return m_max.y() - m_min.y(); }
	inline float depth() const { return m_max.z() - m_min.z(); }

	// Volume of AABB
	inline float volume() const { return width() * height() * depth(); }

	// Check if point inside AABB
	inline bool inside(const Vector3f& p) {
		if (p.x() < m_min.x() || p.x() > m_max.x()) return false;
		if (p.y() < m_min.y() || p.y() > m_max.y()) return false;
		if (p.z() < m_min.z() || p.z() > m_max.z()) return false;
		return true;
	}

	// Check if this AABB overlap with another
	inline bool overlap(const AABB& b) const {
		if (m_min.x() > b.m_max.x()) return false;
		if (m_min.y() > b.m_max.y()) return false;
		if (m_min.z() > b.m_max.z()) return false;

		if (m_max.x() < b.m_min.x()) return false;
		if (m_max.y() < b.m_min.y()) return false;
		if (m_max.z() < b.m_min.z()) return false;
		return true;
	}

private:
	Vector3f m_min;
	Vector3f m_max;
};

}