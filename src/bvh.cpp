#include <numeric> 
#include <stack>

#include <pt/ray.h>
#include <pt/aabb.h>
#include <pt/shape.h>
#include <pt/mesh.h>
#include <pt/bvh.h>
#include <pt/timer.h>

namespace pt {

void BVHTree::build() {
    cout << "Building BVH tree for accelration ...";
    cout.flush();
    Timer timer;

    std::vector<AABB> prim_aabbs(m_shapes->size());
    std::vector<Vector3f> prim_centers(m_shapes->size());

    for (size_t i = 0; i < m_shapes->size(); i++) {
        prim_aabbs[i] = (*m_shapes)[i]->getAABB();
        prim_centers[i] = (*m_shapes)[i]->getCenter();
    }

    BVHTreeBuilder builder(this);
    builder.build(prim_aabbs, prim_centers);

    cout << "done. (took " << timer.elapsedString() << ")" << endl;
    cout << "BVH tree contains " << m_nodes.size() << " nodes!" << endl;
}

bool BVHTree::rayIntersect(const Ray& ray_, Intersection& its) {
    bool intersect = false;

	Ray ray(ray_);
    std::vector<size_t> stack;
    stack.push_back(0);

    while (!stack.empty()) {
        size_t node_idx = stack.back();
        stack.pop_back();
        const Node& node = m_nodes[node_idx];

        if (node.isLeaf()) {
            for (size_t prim_idx = node.first_id, i = 0; i < node.prim_count; prim_idx++, i++) {
                Triangle* primitive = (*m_shapes) [m_prim_ids[prim_idx]];
                Vector3f bary; float t;
                if (primitive->intersect(ray, bary, t)) {
                    ray.max_dis = t; // find nearest intersection point
					intersect = true;
					its.setInfo(primitive, bary);
				}
            }
        }
        else {
            if (node.aabb.intersect(ray)) {
                stack.push_back(node.first_id);
                stack.push_back(node.first_id + 1);
            }
        }
    }

    if (intersect) its.complete();
    return intersect;
}

bool BVHTree::rayIntersect(const Ray& ray) {
    std::vector<size_t> stack;
    stack.push_back(0);

    while (!stack.empty()) {
        size_t node_idx = stack.back();
        stack.pop_back();
        const Node& node = m_nodes[node_idx];

        if (node.isLeaf()) {
            for (size_t prim_idx = node.first_id, i = 0; i < node.prim_count; prim_idx++, i++) {
                Triangle* primitive = (*m_shapes)[m_prim_ids[prim_idx]];
                Vector3f bary; float t;
                if (primitive->intersect(ray, bary, t)) return true;
            }
        }
        else {
            if (node.aabb.intersect(ray)) {
                stack.push_back(node.first_id);
                stack.push_back(node.first_id + 1);
            }
        }
    }

    return false;
}

void BVHTreeBuilder::build(const std::vector<AABB>& aabbs, const std::vector<Vector3f>& centers) {
    const auto prim_count = aabbs.size();
    m_aabbs = aabbs;

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
        const WorkItem& item = stack.top();
        stack.pop();

        auto& node = m_bvh->m_nodes[item.node_id];
        if (item.size() > MinLeafSize) {
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
                if (first_bbox.halfSurfaceArea() < second_bbox.halfSurfaceArea()) {
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

std::optional<size_t> BVHTreeBuilder::trySplit(const AABB& aabb, size_t begin, size_t end) {
    // Find the best split over all axes
    auto leaf_cost = computeNoSplitCost(begin, end, aabb);
    Split best_split = Split{ (begin + end + 1) / 2, leaf_cost, 0 };
    for (size_t axis = 0; axis < 3; ++axis)
        findBestSplit(axis, begin, end, best_split);

    // Make sure that the split is good before proceeding with it
    if (best_split.cost >= leaf_cost) {
        if (end - begin <= MaxLeafSize)
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
            m_accum[i] = right_cost = computeLeafCost(i, end, right_bbox);
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
        auto left_cost = computeLeafCost(begin, i + 1, left_bbox);
        auto cost = left_cost + m_accum[i + 1];
        if (cost < best_split.cost)
            best_split = Split{ i + 1, cost, axis };
        else if (left_cost > best_split.cost)
            break;
    }
}

}