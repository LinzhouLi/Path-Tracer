#pragma once

#include <pt/common.h>
#include <pt/aabb.h>

namespace pt {

// Brute force method
class Accel {
public:
	Accel(std::vector<Triangle*>* primitives) : m_primitives(primitives) { }

	virtual void build();

	virtual bool rayIntersect(const Ray& ray, Intersaction& its);

protected:
	std::vector<Triangle*>* m_primitives;
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

		void makeLeaf(const std::vector<uint32_t>& ids, const AABB& b) {
			prim_ids = ids;
			aabb = b;
		}

		void makeInterior(Node* l, Node* r, const AABB& b) {
			left = l;
			right = r;
			aabb = b;
		}
	};

	BVHTree(std::vector<Triangle*>* primitives) : Accel(primitives) { }

	~BVHTree() {
		for (auto p : node_pool) delete p;
	}

	void build();

	bool rayIntersect(const Ray& ray, Intersaction& its);

private:
	Node* buildRecursive(std::vector<uint32_t>& primitiveIndices, const std::vector<AABB>& prim_aabbs);

	Node* root = nullptr;
	std::vector<Node*> node_pool;
};

}