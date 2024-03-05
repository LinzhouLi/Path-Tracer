#pragma once

#include <pt/accel.h>
#include <pt/mesh.h>
#include <pt/ray.h>
#include <pt/aabb.h>
#include <pt/shape.h>
#include <pt/timer.h>

#include <tbb/tbb.h>
#include <tbb/task_group.h>

namespace pt {

void Accel::build() {
    cout << "Brute force intersection search. No accelration!" << endl;
}

bool Accel::rayIntersect(const Ray& ray, Intersection& its) {
    bool intersect = false;

    Ray ray_(ray);

    for (uint32_t idx = 0; idx < m_shapes->size(); ++idx) {
        Triangle* primitive =  (*m_shapes)[idx];
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

bool Accel::rayIntersect(const Ray& ray) {
    for (uint32_t idx = 0; idx < m_shapes->size(); ++idx) {
        Triangle* primitive = (*m_shapes)[idx];
        Vector3f bary; float t;
        if (primitive->intersect(ray, bary, t)) {
            return true;
        }
    }
    return false;
}


void BVHTree::build() {
    cout << "Building BVH tree for accelration ...";
    cout.flush();
    Timer timer;

    std::vector<AABB> prim_aabbs(m_shapes->size());
    std::vector<uint32_t> prim_ids(m_shapes->size());

    for (uint32_t i = 0; i < m_shapes->size(); i++) {
        prim_aabbs[i] = (*m_shapes)[i]->getAABB();
        prim_ids[i] = i;
    }

    root = buildRecursive(prim_ids.begin(), prim_ids.end(), prim_aabbs);

    cout << "done. (took " << timer.elapsedString() << ")" << endl;
    cout << "BVH tree contains " << node_pool.size() << " nodes!" << endl;
}

struct SplitBucket {
    uint32_t count = 0;
    AABB aabb;
};

BVHTree::Node* BVHTree::buildRecursive(
    std::vector<uint32_t>::iterator prim_ids_begin,
    std::vector<uint32_t>::iterator prim_ids_end,
    const std::vector<AABB>& prim_aabbs
) {
    if (prim_ids_end == prim_ids_begin)
        return nullptr;

    auto prims_num = prim_ids_end - prim_ids_begin;

    Node* node;
    {
        tbb::mutex::scoped_lock lock(m_mutex);
        node = new Node();
        node_pool.push_back(node);
        // if (node_pool.size() % 5000 == 0) cout << "Create " << node_pool.size() << " BVH nodes..." << endl;
    }

    // compute sub-tree aabb
    AABB aabb, centroid_aabb;
    for (auto iter = prim_ids_begin; iter != prim_ids_end; iter++) {
        uint32_t prim_idx = *iter;
        aabb += prim_aabbs[prim_idx];
        centroid_aabb += prim_aabbs[prim_idx].center();
    }

    if (prims_num == 1 || aabb.surfaceArea() == 0.0) {
        node->makeLeaf(prim_ids_begin, prim_ids_end, aabb);
        return node;
    }

    // surface area heuristics
    uint32_t axis = centroid_aabb.getMaxAxis();
    std::vector<uint32_t>::iterator mid;

    if (prims_num <= 2) {
        mid = prim_ids_begin + prims_num / 2;
        std::nth_element( // partial sort
            prim_ids_begin, mid, prim_ids_end,
            [&](uint32_t a, uint32_t b) {
				return prim_aabbs[a].center()[axis] < prim_aabbs[b].center()[axis];
			}
        );
    }
    else {
        // initialze buckets
        constexpr int num_buckets = 12;
        SplitBucket buckets[num_buckets];
        for (auto iter = prim_ids_begin; iter != prim_ids_end; iter++) {
            uint32_t prim_idx = *iter;
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
        float leaf_cost = prims_num * 1.0;
        if (leaf_cost < min_cost) {
            node->makeLeaf(prim_ids_begin, prim_ids_end, aabb);
            return node;
        }

        // partition by split_idx
        mid = std::partition(
            prim_ids_begin, prim_ids_end,
            [=](uint32_t prim_idx) {
                int b_idx = num_buckets * centroid_aabb.offset(prim_aabbs[prim_idx].center())[axis]; // which bucket?
                if (b_idx == num_buckets) b_idx = num_buckets - 1;
                return b_idx <= split_idx;
            }
        );
    }

    // make interior node
    Node* left_node, * right_node;
    if (prims_num > 128 * 1024) {
        tbb::task_group tg;
        tg.run([&] { left_node = buildRecursive(prim_ids_begin, mid, prim_aabbs); });
        tg.run([&] { right_node = buildRecursive(mid, prim_ids_end, prim_aabbs); });
        tg.wait();
    }
    else {
        left_node = buildRecursive(prim_ids_begin, mid, prim_aabbs);
        right_node = buildRecursive(mid, prim_ids_end, prim_aabbs);
    }

    node->makeInterior(left_node, right_node, aabb);
    return node;
}

bool BVHTree::rayIntersect(const Ray& ray, Intersection& its) {
    bool intersect = false;

	Ray ray_(ray);

	std::vector<Node*> stack;
	stack.push_back(root);

    while (!stack.empty()) {
        Node* node = stack.back();
		stack.pop_back();

        if (node == nullptr) continue;

        if (node->isLeaf()) {
            for (uint32_t prim_idx : node->prim_ids) {
				Triangle* primitive = (*m_shapes)[prim_idx];
				Vector3f bary; float t;
                if (primitive->intersect(ray_, bary, t)) {
					ray_.max_dis = t; // find nearest intersection point
					intersect = true;
					its.setInfo(primitive, bary);
				}
			}
        }
        else {
            if (node->aabb.intersect(ray_)) {
				stack.push_back(node->left);
				stack.push_back(node->right);
			}
		}
	}

    if (intersect) {
		its.complete();
	}

	return intersect;
}

bool BVHTree::rayIntersect(const Ray& ray) {
    std::vector<Node*> stack;
    stack.push_back(root);

    while (!stack.empty()) {
        Node* node = stack.back();
        stack.pop_back();

        if (node == nullptr) continue;

        if (node->isLeaf()) {
            for (uint32_t prim_idx : node->prim_ids) {
                Triangle* primitive = (*m_shapes)[prim_idx];
                Vector3f bary; float t;
                if (primitive->intersect(ray, bary, t)) return true;
            }
        }
        else {
            if (node->aabb.intersect(ray)) {
                stack.push_back(node->left);
                stack.push_back(node->right);
            }
        }
    }
    return false;
}
    
}