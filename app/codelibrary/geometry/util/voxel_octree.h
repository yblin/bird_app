//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_H_
#define CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_H_

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/geometry/range_ray_3d.h"
#include "codelibrary/geometry/ray_3d.h"
#include "codelibrary/util/tree/octree.h"

namespace cl {
namespace geometry {

/**
 * Sparse voxel octree.
 */
template <typename T, typename Index>
class VoxelOctree : public Octree<bool, Index> {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using Node = typename Octree<bool, Index>::Node;

    VoxelOctree() = default;

    /**
     * Clear the octree and reset it bounding box and depth.
     */
    void ResetBox(const Box3D<T>& box, int depth) {
        CHECK(!box.empty());
        CHECK(depth > 0);
        CHECK(depth <= (std::numeric_limits<Index>::digits - 1) / 3);

        box_ = box;
        this->Reset(depth);

        voxel_resolution_ = std::max(std::max(box.x_length(), box.y_length()),
                                     box.z_length()) / this->resolution_;
        CHECK(voxel_resolution_ > 0.0);

        size1_ = static_cast<int>(box_.x_length() / voxel_resolution_) + 1;
        size2_ = static_cast<int>(box_.y_length() / voxel_resolution_) + 1;
        size3_ = static_cast<int>(box_.z_length() / voxel_resolution_) + 1;
        size1_ = Clamp(size1_, 1, this->resolution_);
        size2_ = Clamp(size2_, 1, this->resolution_);
        size3_ = Clamp(size3_, 1, this->resolution_);
    }

    /**
     * Reset VoxelOctree to the given surface mesh, i.e., it voxelizes the given
     * mesh.
     *
     * Paramters:
     *  mesh  - the given surface mesh.
     *  depth - the depth of octree.
     */
    template <typename Point>
    void ResetMesh(const SurfaceMesh<Point>& mesh, int depth) {
        static_assert(std::is_same<typename Point::value_type, T>::value, "");

        Array<Point> vertices;
        Array<int> indices;
        mesh.ToTriMesh(&vertices, &indices);
        this->ResetBox(mesh.GetBoundingBox(), depth);

        std::unordered_set<uint64_t> hash;
        for (int v = 0; v + 3 <= indices.size(); v += 3) {
            Triangle3D<T> triangle(vertices[indices[v]],
                                   vertices[indices[v + 1]],
                                   vertices[indices[v + 2]]);
            Box3D<T> box = triangle.bounding_box();
            int lx = GetXIndex(box.x_min());
            int ux = GetXIndex(box.x_max());
            int ly = GetYIndex(box.y_min());
            int uy = GetYIndex(box.y_max());
            int lz = GetZIndex(box.z_min());
            int uz = GetZIndex(box.z_max());
            for (int i = lx; i <= ux; ++i) {
                for (int j = ly; j <= uy; ++j) {
                    for (int k = lz; k <= uz; ++k) {
                        uint64_t c = (uint64_t(i) << 42) | (uint64_t(j) << 21) |
                                      uint64_t(k);
                        if (hash.find(c) != hash.end()) continue;

                        box = GetVoxel(i, j, k);
                        if (Intersect(box, triangle)) {
                            hash.insert(c);
                            this->Insert(i, j, k, true);
                        }
                    }
                }
            }
        }
    }

    /**
     * Insert a voxel at point 'p'.
     */
    template <typename Point>
    std::pair<Node*, bool> InsertVoxel(const Point& p) {
        static_assert(std::is_same<typename Point::value_type, T>::value, "");

        return this->Insert(GetXIndex(p.x), GetYIndex(p.y), GetZIndex(p.z),
                            true);
    }

    /**
     * Return the x-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetXIndex(T v) const {
        int index = static_cast<int>((v - box_.x_min()) / voxel_resolution_);
        return Clamp(index, 0, size1_ - 1);
    }

    /**
     * Return the y-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetYIndex(T v) const {
        int index = static_cast<int>((v - box_.y_min()) / voxel_resolution_);
        return Clamp(index, 0, size2_ - 1);
    }

    /**
     * Return the z-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetZIndex(T v) const {
        int index = static_cast<int>((v - box_.z_min()) / voxel_resolution_);
        return Clamp(index, 0, size3_ - 1);
    }

    /**
     * Get the voxel at leaf node (i, j, k).
     */
    Box3D<T> GetVoxel(int i, int j, int k) const {
        CHECK(i >= 0 && i < this->resolution_);
        CHECK(j >= 0 && j < this->resolution_);
        CHECK(k >= 0 && k < this->resolution_);

        return Box3D<T>(box_.x_min() +  i      * voxel_resolution_,
                        box_.x_min() + (i + 1) * voxel_resolution_,
                        box_.y_min() +  j      * voxel_resolution_,
                        box_.y_min() + (j + 1) * voxel_resolution_,
                        box_.z_min() +  k      * voxel_resolution_,
                        box_.z_min() + (k + 1) * voxel_resolution_);
    }

    /**
     * Get the voxel at node (i, j, k, d).
     */
    Box3D<T> GetVoxel(int i, int j, int k, int d) const {
        CHECK(i >= 0 && i < this->resolution_);
        CHECK(j >= 0 && j < this->resolution_);
        CHECK(k >= 0 && k < this->resolution_);
        CHECK(d >= 0 && d < this->depth_);

        int scale = 1 << (this->depth_ - d - 1);
        return Box3D<T>(box_.x_min() +  i      * scale * voxel_resolution_,
                        box_.x_min() + (i + 1) * scale * voxel_resolution_,
                        box_.y_min() +  j      * scale * voxel_resolution_,
                        box_.y_min() + (j + 1) * scale * voxel_resolution_,
                        box_.z_min() +  k      * scale * voxel_resolution_,
                        box_.z_min() + (k + 1) * scale * voxel_resolution_);
    }

    /**
     * Get the voxel of the given octree node.
     */
    Box3D<T> GetVoxel(const Node* node) const {
        CHECK(node);

        int x, y, z, d;
        node->get_position(&x, &y, &z, &d);
        return GetVoxel(x, y, z, d);
    }

    /**
     * Get the center of voxel(i, j, k).
     */
    Point3D<T> GetVoxelCenter(int i, int j, int k) const {
        CHECK(i >= 0 && i < this->size1_);
        CHECK(j >= 0 && j < this->size2_);
        CHECK(k >= 0 && k < this->size3_);

        return Point3D<T>(box_.x_min() + (i + 0.5) * voxel_resolution_,
                          box_.y_min() + (j + 0.5) * voxel_resolution_,
                          box_.z_min() + (k + 0.5) * voxel_resolution_);
    }

    /**
     * Pick all voxels intersected by the given ray.
     */
    void PickVoxels(const RangeRay3D<T>& ray,
                    Array<const Node*>* voxels) const {
        CHECK(voxels);

        voxels->clear();

        RRangeRay3D r(ray);
        if (!r.Intersect(box_)) return;

        std::queue<std::pair<Node*, RangeRay3D<T>>> rays;
        rays.emplace(this->root_, r);

        while (!rays.empty()) {
            std::pair<Node*, RangeRay3D<T>> p = rays.front();
            rays.pop();

            if (this->is_leaf(p.first)) {
                voxels->push_back(p.first);
                continue;
            }

            for (int i = 0; i < 8; ++i) {
                Node* child = this->GetChild(p.first, i);
                if (!child) continue;

                RangeRay3D<T> r = p.second;
                if (r.Intersect(this->GetVoxel(child))) {
                    rays.emplace(child, r);
                }
            }
        }
    }

    /**
     * Pick all voxels intersected by the given ray.
     */
    void PickVoxels(const RangeRay3D<T>& ray, Array<Node*>* voxels) {
        CHECK(voxels);

        voxels->clear();

        RangeRay3D<T> r(ray);
        if (!r.Intersect(box_)) return;

        std::queue<std::pair<Node*, RangeRay3D<T>>> rays;
        rays.emplace(this->root_, r);

        while (!rays.empty()) {
            std::pair<Node*, RangeRay3D<T>> p = rays.front();
            rays.pop();

            if (this->is_leaf(p.first)) {
                voxels->push_back(p.first);
                continue;
            }

            for (int i = 0; i < 8; ++i) {
                Node* child = this->GetChild(p.first, i);
                if (!child) continue;

                RangeRay3D<T> r = p.second;
                if (r.Intersect(this->GetVoxel(child))) {
                    rays.emplace(child, r);
                }
            }
        }
    }

    /**
     * Pick nearest voxel by ray.
     */
    Node* PickVoxel(const Ray3D<T>& ray) {
        return PickVoxelInternal(ray);
    }

    /**
     * Pick nearest voxel by ray.
     */
    const Node* PickVoxel(const Ray3D<T>& ray) const {
        return PickVoxelInternal(ray);
    }

    const Box3D<T>& box() const {
        return box_;
    }

    T voxel_resolution() const {
        return voxel_resolution_;
    }

private:
    Node* PickVoxelInternal(const Ray3D<T>& ray) const {
        CHECK(this->root_);

        RangeRay3D<T> r(ray);
        if (!r.Intersect(box_)) return nullptr;

        Array<std::pair<Node*, RangeRay3D<T>>> rays;
        rays.emplace_back(this->root_, r);
        T dis = std::numeric_limits<T>::max();

        Node* picked = nullptr;

        while (!rays.empty()) {
            std::pair<Node*, RangeRay3D<T>> p = rays.back();
            rays.pop_back();

            if (this->is_leaf(p.first)) {
                if (p.second.lower() < dis) {
                    dis = p.second.lower();
                    picked = p.first;
                }
                continue;
            }

            for (int i = 0; i < 8; ++i) {
                Node* child = const_cast<Node*>(this->GetChild(p.first, i));
                if (!child) continue;

                RangeRay3D<T> ray = p.second;
                if (ray.lower() < dis && ray.Intersect(this->GetVoxel(child))) {
                    rays.emplace_back(child, ray);
                }
            }
        }
        return picked;
    }

    // Bounding box of the octree.
    Box3D<T> box_;

    // Number of voxels in X, Y and Z.
    int size1_ = 0, size2_ = 0, size3_ = 0;

    // The length of the voxel.
    T voxel_resolution_ = 0;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_H_
