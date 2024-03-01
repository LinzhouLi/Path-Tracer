#include <pt/material.h>
#include <pt/color.h>
#include <pt/bitmap.h>

namespace pt {

Vector3f Material::getBaseColor(const Vector2f& uv) {
	if (m_diffuse_texture) {
		Color3f c = m_diffuse_texture->sample(uv);
		return Vector3f(c.x(), c.y(), c.z());
	}
	else
		return getBaseColor();
}

}