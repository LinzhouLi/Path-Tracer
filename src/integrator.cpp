#pragma once

#include <pt/color.h>
#include <pt/scene.h>
#include <pt/shape.h>
#include <pt/material.h>
#include <pt/integrator.h>

namespace pt {

Color3f GeometryIntegrator::Li(Scene* scene, const Ray& ray) const {
	Intersaction its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f n = its.sha_n;
		n = n.cwiseAbs();
		return Color3f(n.x(), n.y(), n.z());
	}
	else
		return Color3f(0, 0, 0);
}

Color3f BaseColorIntegrator::Li(Scene* scene, const Ray& ray) const {
	Intersaction its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Material* m = scene->getMaterial(its.getShape()->getMaterialId());
		return m->getBaseColor(its.uv);
	}
	else
		return Color3f(0, 0, 0);
}

}