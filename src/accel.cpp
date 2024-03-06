#pragma once

#include <stack>
#include <numeric> 

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
    


inline AABB BVHTreeBuilder::computeAABB(size_t begin, size_t end) const {
    const auto& prim_ids = m_prim_ids[0];
    AABB aabb;
    for (size_t i = begin; i < end; ++i) aabb += m_aabbs[prim_ids[i]];
    return aabb;
}

void BVHTreeBuilder::build(const std::vector<AABB>& aabbs, const std::vector<Vector3f>& centers) {
    const auto prim_count = aabbs.size();
    m_aabbs = aabbs;
    m_centers = centers;

    m_marks.resize(aabbs.size());
    m_accum.resize(aabbs.size());
    for (int axis = 0; axis < 3; ++axis) {
        m_prim_ids[axis].resize(aabbs.size());
        std::iota(m_prim_ids[axis].begin(), m_prim_ids[axis].end(), 0);
        std::sort(m_prim_ids[axis].begin(), m_prim_ids[axis].end(), [&](size_t i, size_t j) {
            return centers[i][axis] < centers[j][axis];
        });
    }

    m_bvh->m_nodes.reserve(2 * prim_count);
    m_bvh->m_nodes.emplace_back();
    m_bvh->m_nodes.back().setAABB(computeAABB(0, prim_count));

    std::stack<WorkItem> stack;
    stack.push(WorkItem{ 0, 0, prim_count });

    while (!stack.empty()) {
        auto item = stack.top();
        stack.pop();

        auto& node = m_bvh->m_nodes[item.node_id];
        if (item.size() > min_leaf_size) {
            if (auto split_pos = trySplit(node.aabb, item.begin, item.end)) {
                size_t first_child = m_bvh->m_nodes.size();
                node.makeInterior(first_child);

                m_bvh->m_nodes.resize(first_child + 2);

                AABB first_bbox = computeAABB(item.begin, *split_pos);
                AABB second_bbox = computeAABB(*split_pos, item.end);
                auto first_range = std::make_pair(item.begin, *split_pos);
                auto second_range = std::make_pair(*split_pos, item.end);

                // For "any-hit" queries, the left child is chosen first, so we make sure that
                // it is the child with the largest area, as it is more likely to contain an
                // an occluder. See "SATO: Surface Area Traversal Order for Shadow Ray Tracing",
                // by J. Nah and D. Manocha.
                if (first_bbox.surfaceArea() < second_bbox.surfaceArea()) {
                    std::swap(first_bbox, second_bbox);
                    std::swap(first_range, second_range);
                }

                WorkItem first_item = WorkItem{ first_child + 0, first_range.first, first_range.second };
                WorkItem second_item = WorkItem{ first_child + 1, second_range.first, second_range.second };
                m_bvh->m_nodes[first_child + 0].setAABB(first_bbox);
                m_bvh->m_nodes[first_child + 1].setAABB(second_bbox);

                // Process the largest child item first, in order to minimize the stack size.
                if (first_item.size() < second_item.size())
                    std::swap(first_item, second_item);

                stack.push(first_item);
                stack.push(second_item);
                continue;
            }
        }

        node.makeLeaf(item.begin, item.size());
    }

    m_bvh->m_prim_ids = std::move(m_prim_ids[0]);
    m_bvh->m_nodes.shrink_to_fit();
}

inline void BVHTreeBuilder::mark_primitives(size_t axis, size_t begin, size_t split_pos, size_t end) {
    for (size_t i = begin; i < split_pos; ++i) m_marks[m_prim_ids[axis][i]] = true;
    for (size_t i = split_pos; i < end; ++i)   m_marks[m_prim_ids[axis][i]] = false;
}

std::optional<size_t> BVHTreeBuilder::trySplit(const AABB& aabb, size_t begin, size_t end) {
    // Find the best split over all axes
    auto leaf_cost = sah.get_non_split_cost(begin, end, aabb);
    auto best_split = Split{ (begin + end + 1) / 2, leaf_cost, 0 };
    for (size_t axis = 0; axis < 3; ++axis)
        findBestSplit(axis, begin, end, best_split);

    // Make sure that the split is good before proceeding with it
    if (best_split.cost >= leaf_cost) {
        if (end - begin <= max_leaf_size)
            return std::nullopt;

        // If the number of primitives is too high, fallback on a split at the
        // median on the largest axis.
        best_split.pos = (begin + end + 1) / 2;
        best_split.axis = aabb.getMaxAxis();
    }

    // Partition primitives (keeping the order intact so that the next recursive calls do not
    // need to sort primitives again).
    mark_primitives(best_split.axis, begin, best_split.pos, end);
    for (size_t axis = 0; axis < 3; ++axis) {
        if (axis == best_split.axis)
            continue;
        std::stable_partition(
            m_prim_ids[axis].begin() + begin,
            m_prim_ids[axis].begin() + end,
            [&](size_t i) { return m_marks[i]; });
    }

    return std::make_optional(best_split.pos);
}

void BVHTreeBuilder::findBestSplit(size_t axis, size_t begin, size_t end, Split& best_split) {
    size_t first_right = begin;

    // Sweep from the right to the left, computing the partial SAH cost
    AABB right_bbox;
    for (size_t i = end - 1; i > begin;) {
        static constexpr size_t chunk_size = 32;
        size_t next = i - std::min(i - begin, chunk_size);
        float right_cost = 0.0f;
        for (; i > next; --i) {
            right_bbox += m_aabbs[m_prim_ids[axis][i]];
            m_accum[i] = right_cost = sah.get_leaf_cost(i, end, right_bbox);
        }
        // Every `chunk_size` elements, check that we are not above the maximum cost
        if (right_cost > best_split.cost) {
            first_right = i;
            break;
        }
    }

    // Sweep from the left to the right, computing the full cost
    AABB left_bbox;
    for (size_t i = begin; i < first_right; ++i)
        left_bbox += m_aabbs[m_prim_ids[axis][i]];
    for (size_t i = first_right; i < end - 1; ++i) {
        left_bbox += m_aabbs[m_prim_ids[axis][i]];
        auto left_cost = sah.get_leaf_cost(begin, i + 1, left_bbox);
        auto cost = left_cost + m_accum[i + 1];
        if (cost < best_split.cost)
            best_split = Split{ i + 1, cost, axis };
        else if (left_cost > best_split.cost)
            break;
    }
}

}