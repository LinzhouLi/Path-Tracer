#pragma once

#include <pt/accel.h>
#include <pt/mesh.h>
#include <pt/ray.h>
#include <pt/aabb.h>
#include <pt/shape.h>

namespace pt {

void Accel::build() {
    cout << "Brute force intersection search. No accelration!" << endl;
}

bool Accel::rayIntersect(const Ray& ray, Intersaction& its) {
    bool intersect = false;

    Ray ray_(ray);

    for (uint32_t idx = 0; idx < m_primitives->size(); ++idx) {
        Triangle* primitive =  (*m_primitives)[idx];
        Vector3f bary; float t;
        if (primitive->intersect(ray, bary, t)) {
            ray_.max_dis = t; // find nearest intersection point
            intersect = true;
            its.setInfo(primitive, bary);
        }
    }

    if (intersect) {
        its.complete();
    }

    return intersect;
}

void BVHTree::build() {
    cout << "Building BVH tree for accelration ..." << endl;
    aabbs.resize(m_primitives->size());

    std::vector<int> triangleIndices;
    for (uint32_t i = 0; i < m_primitives->size(); i++) {
        aabbs[i] = (*m_primitives)[i]->getAABB();
    }
}

bool BVHTree::rayIntersect(const Ray& ray, Intersaction& its) {
    return true;
}
    
}