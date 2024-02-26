#pragma once

#include <pt/vector.h>
#include <pt/ray.h>

namespace pt {

class Transform {
public:
	enum Type { Vector, Scaler, Normal };

	Transform(const Eigen::Matrix4f& matrix = Eigen::Matrix4f::Identity()) : m_matrix(matrix) {
		m_inverse = matrix.inverse();
	}

	void setMatrix(const Eigen::Matrix4f& matrix) {
		m_matrix = matrix;
		m_inverse = matrix.inverse();
	}

	const Eigen::Matrix4f& getMatrix() { return m_matrix; }

	const Eigen::Matrix4f& getInverseMatrix() { return m_inverse; }

	Vector3f apply(const Vector3f& vec, Type type = Type::Vector) {
		if (type == Type::Vector) {
			return m_matrix.topLeftCorner<3, 3>() * vec;
		}
		else if (Type::Scaler) {
			Vector4f result = m_matrix * Vector4f(vec.x(), vec.y(), vec.z(), 1.0f);
			return result.head<3>() / result.w();
		}
		else if (Type::Normal) {
			return m_inverse.topLeftCorner<3, 3>().transpose() * vec;
		}
		else
			throw("Transform calculation error!");
	}

	Vector4f apply(const Vector4f& vec) {
		return m_matrix * vec;
	}

	Ray apply(const Ray& ray) {
		return Ray(
			apply(ray.org, Type::Scaler),
			apply(ray.dir, Type::Vector),
			ray.min_dis, ray.max_dis
		);
	}

private:
	Eigen::Matrix4f m_matrix;
	Eigen::Matrix4f m_inverse;
};

}