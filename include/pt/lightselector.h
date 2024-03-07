#pragma once

#include <pt/common.h>

namespace pt {

class UniformLightSelector {
public:
	UniformLightSelector(std::vector<AreaLight*>* lights) : m_lights(lights) { }

	AreaLight* select(float u) const {
		return m_lights->at(static_cast<int>(u * m_lights->size()));
	}

	float pdf(const AreaLight* light) const {
		return 1.0f / m_lights->size();
	}

private:
	std::vector<AreaLight*>* m_lights;

};

}