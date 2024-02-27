#pragma once

#include <pt/accel.h>
#include <pt/mesh.h>
#include <pt/ray.h>

namespace pt {

void Accel::addMesh(Mesh* mesh) {
    if (m_mesh)
        throw PathTracerException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
}

void Accel::build() {
    cout << "Brute force search. No accelration!" << endl;
}

bool Accel::rayIntersect(const Ray& ray, Intersaction& its) {
    bool intersect = false;

    Ray ray_(ray);

    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        Vector3f bary; float t;
        if (rayTriangleIntersect(m_mesh->getTriangle(idx), ray_, bary, t)) {
            ray_.max_dis = t; // find nearest intersection point
            intersect = true;
            its.setInfo(m_mesh, idx, bary);
        }
    }

    if (intersect) {
        its.complete();
    }

    return intersect;
}

void BVHTree::build() {
    cout << "Build BVH tree for accelration!" << endl;
}

bool BVHTree::rayIntersect(const Ray& ray, Intersaction& its) {
    return true;
}
    
}