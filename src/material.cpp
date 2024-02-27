#include <pt/material.h>
#include <pt/color.h>
#include <pt/bitmap.h>

namespace pt {

Color3f Material::getBaseColor(const Vector2f& uv) {
	if (m_diffuse_texture)
		return m_diffuse_texture->sample(uv);
	else
		return getBaseColor();
}

}