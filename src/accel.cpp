#pragma once

#include <pt/accel.h>
#include <pt/mesh.h>
#include <pt/ray.h>
#include <pt/aabb.h>

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
    cout << "Building BVH tree for accelration ..." << endl;
    uint32_t numTriangles = m_mesh->getTriangleCount();
    aabbs.resize(numTriangles);

    std::vector<int> triangleIndices;
    for (uint32_t i = 0; i < numTriangles; i++) {
        triangleIndices.push_back(i);
        const Mesh::TriVertex& triangle = m_mesh->getTriangle(i);
        AABB aabb(triangle[0], triangle[1], triangle[2]);
        aabbs[i] = aabb;
    }
}

bool BVHTree::rayIntersect(const Ray& ray, Intersaction& its) {
    return true;
}
    
}