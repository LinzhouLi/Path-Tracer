#pragma once

#include <optional>

#include <pt/common.h>
#include <pt/aabb.h>
#include <tbb/mutex.h>

namespace pt {

// Brute force method
class Accel {
public:
	Accel(std::vector<Triangle*>* primitives) : m_shapes(primitives) { }

	virtual void build();

	virtual bool rayIntersect(const Ray& ray, Intersection& its);

	virtual bool rayIntersect(const Ray& ray);

protected:
	std::vector<Triangle*>* m_shapes;
};

// BVH tree accelration
class BVHTree : public Accel {
public:
	class Node {
	public:
		AABB aabb;
		Node* left = nullptr;
		Node* right = nullptr;
		std::vector<uint32_t> prim_ids;

		void makeLeaf(
			std::vector<uint32_t>::iterator prim_ids_begin,
			std::vector<uint32_t>::iterator prim_ids_end, 
			const AABB& b
		) {
			prim_ids = std::vector<uint32_t>(prim_ids_begin, prim_ids_end);
			aabb = b;
		}

		void makeInterior(Node* l, Node* r, const AABB& b) {
			left = l;
			right = r;
			aabb = b;
		}

		inline bool isLeaf() const { 
			return left == nullptr && right == nullptr; 
		}
	};

	BVHTree(std::vector<Triangle*>* primitives) : Accel(primitives) { }

	~BVHTree() {
		for (auto p : node_pool) delete p;
	}

	void build();

	bool rayIntersect(const Ray& ray, Intersection& its);

	bool rayIntersect(const Ray& ray);

private:
	Node* buildRecursive(
		std::vector<uint32_t>::iterator prim_ids_begin, 
		std::vector<uint32_t>::iterator prim_ids_end,
		const std::vector<AABB>& prim_aabbs
	);

	Node* root = nullptr;
	std::vector<Node*> node_pool;
	mutable tbb::mutex m_mutex; // lock for node_pool.push_back()
};


class BBVHTree : public Accel {
public:
	friend class BVHTreeBuilder;

	struct Node {
		AABB aabb;
		size_t first_id, prim_count;

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
	};

	std::vector<Node> m_nodes;
	std::vector<size_t> m_prim_ids;
};

class BVHTreeBuilder {
public:
	BVHTreeBuilder(BBVHTree* bvh) : m_bvh(bvh) { }

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

	inline AABB computeAABB(size_t begin, size_t end) const;

	inline void mark_primitives(size_t axis, size_t begin, size_t split_pos, size_t end);

	std::optional<size_t> trySplit(const AABB& bbox, size_t begin, size_t end);

	void findBestSplit(size_t axis, size_t begin, size_t end, Split& best_split);

	BBVHTree* m_bvh;
	std::vector<bool> m_marks;
	std::vector<float> m_accum;
	std::vector<size_t> m_prim_ids[3];
	std::vector<AABB> m_aabbs;
	std::vector<Vector3f> m_centers;
};

}