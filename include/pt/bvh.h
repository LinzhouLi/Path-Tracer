#pragma once

#include <pt/common.h>
#include <pt/aabb.h>
#include <pt/accel.h>

namespace pt {

class BVHTree : public Accel {
public:
	friend class BVHTreeBuilder;

	struct Node {
		AABB aabb;
		size_t first_id, prim_count;

		static inline bool isLeftSibling(size_t node_id) { return node_id % 2 == 1; }

		static inline size_t getSiblingId(size_t node_id) {
			return isLeftSibling(node_id) ? node_id + 1 : node_id - 1;
		}

		static inline size_t getLeftSiblingId(size_t node_id) {
			return isLeftSibling(node_id) ? node_id : node_id - 1;
		}

		static inline size_t getRightSiblingId(size_t node_id) {
			return isLeftSibling(node_id) ? node_id + 1 : node_id;
		}

		inline void makeLeaf(size_t first_prim, size_t prim_count) {
			assert(prim_count > 0);
			this->first_id = first_prim;
			this->prim_count = prim_count;
		}

		inline void makeInterior(size_t first_child) {
			this->prim_count = 0;
			this->first_id = first_child;
		}

		inline void setAABB(const AABB& aabb) { this->aabb = aabb; }

		inline bool isLeaf() const { return prim_count != 0; }
	};

	BVHTree(std::vector<Triangle*>* primitives) : Accel(primitives) { }
	
	void build();

	bool rayIntersect(const Ray& ray, Intersection& its);

	bool rayIntersect(const Ray& ray);

	std::string toString() const;

private:
	std::vector<Node> m_nodes;
	std::vector<size_t> m_prim_ids;
};


class BVHTreeBuilder {
public:
	static constexpr size_t MinLeafSize = 1;
	static constexpr size_t MaxLeafSize = 8;

	BVHTreeBuilder(BVHTree* bvh) : m_bvh(bvh) { }

	void build(const std::vector<AABB>& aabbs, const std::vector<Vector3f>& centers);

private:
	struct WorkItem {
		size_t node_id;
		size_t begin;
		size_t end;

		inline size_t size() const { return end - begin; }
	};

	struct Split {
		size_t pos;
		float cost;
		size_t axis;
	};

	inline AABB computeAABB(size_t begin, size_t end) const {
		const auto& prim_ids = m_prim_ids[0];
		AABB aabb;
		for (size_t i = begin; i < end; ++i) aabb += m_aabbs[prim_ids[i]];
		return aabb;
	}

	inline float computeNoSplitCost(size_t begin, size_t end, const AABB& aabb) const {
		return aabb.halfSurfaceArea() * (end - begin);
	}

	inline float computeLeafCost(size_t begin, size_t end, const AABB& aabb) const {
		return aabb.halfSurfaceArea() * (end - begin - 1.0f);
	}

	inline void mark_primitives(size_t axis, size_t begin, size_t split_pos, size_t end) {
		for (size_t i = begin; i < split_pos; ++i) m_marks[m_prim_ids[axis][i]] = true;
		for (size_t i = split_pos; i < end; ++i)   m_marks[m_prim_ids[axis][i]] = false;
	}

	std::optional<size_t> trySplit(const AABB& bbox, size_t begin, size_t end);

	void findBestSplit(size_t axis, size_t begin, size_t end, Split& best_split);

	BVHTree* m_bvh;
	std::vector<bool> m_marks;
	std::vector<float> m_accum;
	std::vector<size_t> m_prim_ids[3];
	std::vector<AABB> m_aabbs;
};

}