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
        if (primitive->intersect(ray_, bary, t)) {
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

    std::vector<AABB> prim_aabbs(m_primitives->size());
    std::vector<uint32_t> prim_ids(m_primitives->size());

    for (uint32_t i = 0; i < m_primitives->size(); i++) {
        prim_aabbs[i] = (*m_primitives)[i]->getAABB();
        prim_ids[i] = i;
    }

    root = buildRecursive(prim_ids, prim_aabbs);
    cout << "BVH tree contains " << node_pool.size() << " nodes!" << endl;
}

struct SplitBucket {
    uint32_t count = 0;
    AABB aabb;
};

BVHTree::Node* BVHTree::buildRecursive(std::vector<uint32_t>& prim_ids, const std::vector<AABB>& prim_aabbs) {
    if (prim_ids.size() == 0)
        return nullptr;

    Node* node = new Node();
    node_pool.push_back(node);

    // compute sub-tree aabb
    AABB aabb, centroid_aabb;
    for (uint32_t idx : prim_ids) {
        aabb += prim_aabbs[idx];
        centroid_aabb += prim_aabbs[idx].center();
    }

    if (prim_ids.size() == 1 || aabb.surfaceArea() == 0.0) {
        node->makeLeaf(prim_ids, aabb);
        return node;
    }

    // surface area heuristics
    uint32_t axis = centroid_aabb.getMaxAxis();

    // initialze buckets
    constexpr int num_buckets = 12;
    SplitBucket buckets[num_buckets];
    for (uint32_t prim_idx : prim_ids) {
        int b_idx = num_buckets * centroid_aabb.offset(prim_aabbs[prim_idx].center())[axis]; // which bucket?
        if (b_idx == num_buckets) b_idx = num_buckets - 1;
        buckets[b_idx].count++;
        buckets[b_idx].aabb += prim_aabbs[prim_idx];
    }

    // compute costs for splitting after each bucket
    constexpr int num_splits = num_buckets - 1;
    float costs[num_splits] = {};

    // partially initialize costs using a forward scan over splits
    uint32_t count_below = 0;
    AABB aabb_below;
    for (int i = 0; i < num_splits; i++) {
        aabb_below += buckets[i].aabb;
        count_below += buckets[i].count;
        costs[i] += count_below * aabb_below.surfaceArea();
    }

    // finish initializing costs using a backward scan over splits
    uint32_t count_above = 0;
    AABB aabb_above;
    for (int i = num_splits; i >= 1; i--) {
        aabb_above += buckets[i].aabb;
        count_above += buckets[i].count;
        costs[i - 1] += count_above * aabb_above.surfaceArea();
    }

    // find best split with min cost
    int split_idx = -1;
    float min_cost = std::numeric_limits<float>::infinity();
    for (int i = 0; i < num_splits; i++) {
        if (costs[i] < min_cost) {
            min_cost = costs[i];
            split_idx = i;
        }
    }
    min_cost = 0.5f + min_cost / aabb.surfaceArea();

    // make leaf node
    float leaf_cost = prim_ids.size();
    if (leaf_cost < min_cost) {
        node->makeLeaf(prim_ids, aabb);
        return node;
    }

    // partition by split_idx
    auto mid = std::partition(
        prim_ids.begin(), prim_ids.end(),
        [=](uint32_t prim_idx) {
            int b_idx = num_buckets * centroid_aabb.offset(prim_aabbs[prim_idx].center())[axis]; // which bucket?
            if (b_idx == num_buckets) b_idx = num_buckets - 1;
            return b_idx <= split_idx;
        }
    );

    // make interior node
    Node* left_node = buildRecursive(std::vector<uint32_t>(prim_ids.begin(), mid), prim_aabbs);
    Node* right_node = buildRecursive(std::vector<uint32_t>(mid, prim_ids.end()), prim_aabbs);

    node->makeInterior(left_node, right_node, aabb);
    return node;
}

bool BVHTree::rayIntersect(const Ray& ray, Intersaction& its) {
    return Accel::rayIntersect(ray, its);
}
    
}