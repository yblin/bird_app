//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_TREE_RTREE_H_
#define CODELIBRARY_GEOMETRY_UTIL_TREE_RTREE_H_

#include <cmath>
#include <queue>

#include "codelibrary/base/array.h"
#include "codelibrary/base/pool.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace geometry {

/**
 * Immutable 2D R-Tree used for 2D spatial access methods.
 *
 * Reference:
 *   Scott T. Leutenegger, J. M. Edgington, and Mario A. Lopez. 1997.
 *   STR: A Simple and Efficient Algorithm for R-Tree Packing. In ICDE.
 *   497–506.
 */
template <typename T>
class RTree2D {
    using Box = Box2D<T>;

    struct Node {
        // Parent node.
        Node* parent = nullptr;

        // Bounding box of this node.
        Box box;

        // Center point of the box, used for sorting.
        Point2D<T> center;

        // Child nodes.
        Array<Node*> child_nodes;

        // The index of entry, only works in the leaf nodes.
        int entry = -1;

        bool is_leaf_node() const {
            return child_nodes.empty();
        }
    };

    /**
     * Used to divide points in the horizontal and vertical directions.
     */
    struct CompareXY {
        bool operator() (Node* a, Node* b) const {
            return a->center.x == b->center.x ? a->center.y < b->center.y
                                              : a->center.x < b->center.x;
        }
    };
    struct CompareYX {
        bool operator() (Node* a, Node* b) const {
            return a->center.y == b->center.y ? a->center.x < b->center.x
                                              : a->center.y < b->center.y;
        }
    };

public:
    /**
     * Parameters:
     *  node_capacity - the maximum number of children nodes (16 by default).
     */
    explicit RTree2D(int node_capacity = 16)
        : node_capacity_(node_capacity) {
        CHECK(node_capacity_ > 0);
    }

    RTree2D(const RTree2D&) = delete;

    RTree2D& operator=(const RTree2D&) = delete;

    /**
     * Reconstruct rtree with a set of line segments.
     */
    void Construct(const Array<Segment2D<T>>& segs) {
        Array<Box2D<T>> boxes(segs.size());
        for (int i = 0; i < segs.size(); ++i) {
            boxes[i] = segs[i].bounding_box();
        }
        Construct(boxes);
    }

    /**
     * Reconstruct rtree with a set of bounding boxes.
     */
    void Construct(const Array<Box2D<T>>& boxes) {
        this->clear();
        if (boxes.empty()) return;

        for (auto b : boxes) {
            CreateLeafNode(b);
        }

        int start = 0, end = nodes_.size();
        while (true) {
            STRPack(start, end);
            if (end + 1 == nodes_.size()) break;
            start = end;
            end = nodes_.size();
        }
        root_ = nodes_.back();
    }

    /**
     * Find entries intersected with the given box.
     */
    void Find(const Box& range, Array<int>* entries) const {
        CHECK(entries);

        entries->clear();
        if (!root_ || range.empty()) return;

        std::queue<const Node*> q;
        q.push(root_);

        while (!q.empty()) {
            const Node* node = q.front();
            q.pop();

            if (!Intersect(node->box, range)) continue;

            if (node->is_leaf_node()) {
                entries->push_back(node->entry);
                continue;
            }

            for (auto child : node->child_nodes) {
                q.push(child);
            }
        }
    }

    bool empty() const {
        return root_ == nullptr;
    }

    void clear() {
        root_ = nullptr;
        nodes_.clear();
        pool_.clear();
    }

    Node* root() const {
        return root_;
    }

    const Array<Node*>& nodes() const {
        return nodes_;
    }

private:
    /**
     * Create a leaf node.
     */
    Node* CreateLeafNode(const Box& box) {
        Node* node = pool_.Allocate();
        node->parent = nullptr;
        node->box = box;
        node->center = Center(node->box);
        node->entry = nodes_.size();
        node->child_nodes.clear();
        nodes_.push_back(node);
        return node;
    }

    /**
     * Create a non-leaf node.
     */
    Node* CreateNode(int first, int last) {
        CHECK(first != last);
        CHECK(last - first <= node_capacity_);

        Node* node = pool_.Allocate();
        node->parent = nullptr;
        node->box = Box();
        node->entry = -1;
        node->child_nodes.clear();
        nodes_.push_back(node);

        for (int i = first; i < last; ++i) {
            nodes_[i]->parent = node;
            node->box.Union(nodes_[i]->box);
            node->child_nodes.push_back(nodes_[i]);
        }
        node->center = Center(node->box);

        return node;
    }

    /**
     * Sort-tile-recursive packing technique for [first, last) nodes.
     */
    void STRPack(int first, int last) {
        CHECK(first >= 0 && first < last);
        CHECK(last <= nodes_.size());

        int n = last - first;
        if (n < node_capacity_) {
            CreateNode(first, last);
            return;
        }

        int m = (n + node_capacity_ - 1) / node_capacity_;
        int sqrt_m = std::ceil(std::sqrt(m));
        CHECK(sqrt_m >= 1);

        std::sort(nodes_.begin() + first, nodes_.begin() + last, CompareXY());

        int base_size = n / sqrt_m;
        int remainder = n % sqrt_m;
        int start = first;
        for (int i = 0; i < sqrt_m; ++i) {
            int size = base_size + (i < remainder ? 1 : 0);
            int end = start + size;

            std::sort(nodes_.begin() + start, nodes_.begin() + end, CompareYX());
            for (int l = start; l < end; l += node_capacity_) {
                int r = std::min(l + node_capacity_, end);
                CreateNode(l, r);
            }

            start = end;
        }
    }

    Node* root_ = nullptr;
    int node_capacity_;
    Array<Node*> nodes_;
    Pool<Node> pool_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_TREE_RTREE_H_
