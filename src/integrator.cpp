#pragma once

#include <pt/color.h>
#include <pt/scene.h>
#include <pt/integrator.h>

namespace pt {

void GeometryIntegrator::preprocess(const Scene* scene) {
	cout << "Build GeometryIntegrator!" << endl;
}

Color3f GeometryIntegrator::Li(const Scene* scene, const Ray& ray) const {
	bool hit = scene->rayIntersect(ray);
	if (hit)
		return Color3f(0, 1, 0);
	else
		return Color3f(1, 1, 1);
}

}