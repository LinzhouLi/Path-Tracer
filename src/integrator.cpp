#pragma once

#include <pt/color.h>
#include <pt/scene.h>
#include <pt/mesh.h>
#include <pt/material.h>
#include <pt/integrator.h>

namespace pt {

Color3f GeometryIntegrator::Li(Scene* scene, const Ray& ray) const {
	Intersaction its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Vector3f n = its.normal;
		n = n.cwiseAbs();
		return Color3f(n.x(), n.y(), n.z());
	}
	else
		return Color3f(1, 1, 1);
}

Color3f BaseColorIntegrator::Li(Scene* scene, const Ray& ray) const {
	Intersaction its;
	bool hit = scene->rayIntersect(ray, its);
	if (hit) {
		Material* m = scene->getMaterial(its.mat_id);
		return m->getBaseColor(its.uv);
	}
	else
		return Color3f(1, 1, 1);
}

}