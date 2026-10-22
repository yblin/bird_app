//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_TOPOLOGY_LAND_MESH_H_
#define CODELIBRARY_GIS_TOPOLOGY_LAND_MESH_H_

#include <set>
#include <map>

#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/util/flat/append_only_array.h"

namespace cl {
namespace gis {

/**
 * Multi-level mesh structure designed for land regions.
 *
 * It represents multi-level regions (e.g., overpass roads, and other layered
 * geographic features) through triangles and their topological connectivity,
 * which cannot be directly represented by 2D meshes.
 */
template <typename T>
class LandMesh {
public:
    using Point = Point2D<T>;

    /**
     * Edge of each triangle.
     */
    struct Edge {
        // ID of this edge.
        int id;
        // Source and target points of the edge.
        Point source, target;
        // Triangles that contains this edge.
        Array<int> triangles;
    };

    /**
     * Triangle structure.
     */
    struct Triangle : public Triangle2D<T> {
        // ID of this triangle.
        int id;

        // Vertical level of this triangle.
        int level = 0;

        // Triangle edges.
        Array<int> edges;

        Triangle() = default;

        Triangle(const Triangle2D<T>& triangle)
            : Triangle2D<T>(triangle) {}
    };

    LandMesh() = default;

    void clear() {
        edges_from_.clear();
        edges_.clear();
        triangles_.clear();
    }

    /**
     * Insert a triangle into the specified level of the mesh.
     *
     * Return the ID of the inserted triangle in the mesh; or -1 if the input is
     * invalid.
     */
    int InsertTriangle(const Triangle2D<T>& triangle, int level) {
        CHECK(level >= 0);

        if (triangle.empty()) return -1;
        if (triangle.vertices()[0] == triangle.vertices()[1] ||
            triangle.vertices()[0] == triangle.vertices()[2] ||
            triangle.vertices()[1] == triangle.vertices()[2]) return -1;

        int e1 = InsertEdge(triangle.vertices()[0], triangle.vertices()[1]);
        int e2 = InsertEdge(triangle.vertices()[1], triangle.vertices()[2]);
        int e3 = InsertEdge(triangle.vertices()[2], triangle.vertices()[0]);

        int id = triangles_.size();
        Triangle triangle_node(triangle);
        triangle_node.id = id;
        triangle_node.level = level;
        triangle_node.edges = {e1, e2, e3};
        triangles_.push_back(triangle_node);

        edges_[e1].triangles.push_back(id);
        edges_[e2].triangles.push_back(id);
        edges_[e3].triangles.push_back(id);

        return id;
    }

    /**
     * Find the edge (source, target), or return -1 if it does not exist.
     */
    int FindEdge(const Point& source, const Point& target) const {
        if (edges_from_.find(source) == edges_from_.end()) return -1;

        for (int eid : edges_from_.at(source)) {
            const Edge& e = edges_[eid];
            if (e.target == target) return eid;
        }
        return -1;
    }

    /**
     * Find the twin edge of 'e', or return -1 if it does not exist.
     */
    int TwinEdge(const Edge& e) const {
        return FindEdge(e.target, e.source);
    }

    // O(1) member functions.
    bool empty()      const { return triangles_.empty();       }
    int n_edges()     const { return edges_.active_size();     }
    int n_triangles() const { return triangles_.active_size(); }

    /**
     * Return eid-th edge.
     */
    const Edge& edge(int eid) const {
        CHECK(edges_.contain(eid));
        return edges_[eid];
    }

    bool contain_edge(int edge) const {
        return edges_.contain(edge);
    }

    bool contain_triangle(int triangle) const {
        return triangles_.contain(triangle);
    }

    const AppendOnlyArray<Edge>& edges() const {
        return edges_;
    }

    const AppendOnlyArray<Triangle>& triangles() const {
        return triangles_;
    }

private:
    /**
     * Insert a directed edge from point 's' to point 't'.
     *
     * If an edge already exists between these two vertices, the function
     * returns the index of the existing edge instead of creating a new one.
     */
    int InsertEdge(Point s, Point t) {
        CHECK(s != t);

        int eid = this->FindEdge(s, t);
        if (eid != -1) return eid;

        Edge e;
        e.id = edges_.size();
        e.source = s;
        e.target = t;
        edges_from_[s].push_back(e.id);
        edges_.push_back(e);

        return e.id;
    }

    // edges_from_[p] stores all edges starting from the point 'p'.
    std::map<Point, Array<int>> edges_from_;

    // Edge list.
    AppendOnlyArray<Edge> edges_;

    // Triangle list.
    AppendOnlyArray<Triangle> triangles_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_TOPOLOGY_LAND_MESH_H_
