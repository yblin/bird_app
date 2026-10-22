//
// Copyright 2011-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_

#include <queue>

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D polygon boolean operations.
 *
 * This class depends on Arrangment2D.
 */
template <typename T>
class PolygonBoolean2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;
    using Mesh = HalfedgeList<Point>;
    using Vertex = typename Mesh::Vertex;
    using Halfedge = typename Mesh::Halfedge;

    // Color flag for mesh halfedges.
    enum Color {
        BLACK  = 0x00,
        RED    = 0x01,
        BLUE   = 0x02,
        PURPLE = 0x03
    };

public:
    /**
     * A suitable snap threshold is important for maintaining the topology of
     * polygons. Otherwise topological errors may occur.
     */
    PolygonBoolean2D(const MultiPolygon2D<T>& red_polygon,
                     const MultiPolygon2D<T>& blue_polygon,
                     double threshold = 0.0)
        : threshold_(threshold), arrangement_(threshold),
          red_polygon_(red_polygon), blue_polygon_(blue_polygon) {
        CHECK(threshold_ >= 0.0);

        Initialize();
    }

    PolygonBoolean2D(const PolygonBoolean2D&) = delete;

    PolygonBoolean2D& operator=(const PolygonBoolean2D&) = delete;

    /**
     * Get the union of two polygons.
     */
    void Union(MultiPolygon2D<T>* result_polygon) {
        GetResult(BLACK, result_polygon);
    }

    /**
     * Get the intersection of two polygons.
     */
    void Intersection(MultiPolygon2D<T>* result_polygon) {
        GetResult(PURPLE, result_polygon);
    }

    /**
     * Get the difference of two polygons (Red polygon - Blue polygon).
     */
    void Difference(MultiPolygon2D<T>* result_polygon) {
        GetResult(RED, result_polygon);
    }

protected:
    /**
     * Get the boolean results by given color.
     */
    void GetResult(int color, MultiPolygon2D<T>* result_polygon) const {
        CHECK(result_polygon);
        result_polygon->clear();

        auto flags = halfedge_list_.AddHalfedgeProperty(false);
        Array<Point2D<T>> points;
        for (const Halfedge* e : halfedge_list_) {
            if (flags[e]) continue;

            if (color_[e] == color) {
                points.clear();
                for (const auto& e1 : halfedge_list_.circular_list(e)) {
                    flags[e1] = true;
                    points.push_back(e1->source_point());
                }

                Polygon2D<T> polygon(points);
                if (polygon.empty()) continue;

                if (color == BLACK) {
                    result_polygon->Insert(polygon, polygon.IsClockwise());
                } else {
                    result_polygon->Insert(polygon, !polygon.IsClockwise());
                }
            }
        }
    }

    /**
     * Triangulation.
     */
    void Initialize() {
        Triangulation();
        if (halfedge_list_.empty()) return;

        const auto& triangulation = arrangement_.triangulation();

        // Step 1. Deal the coincide edge and find the outside boundary of two
        // polygons.
        color_ = halfedge_list_.AddHalfedgeProperty("color", 0);
        for (Halfedge* e : halfedge_list_) {
            color_[e] = triangulation.color(e);
        }
        Halfedge* out_boundary = nullptr;
        for (Halfedge* e : halfedge_list_) {
            int t = color_[e] & color_[e->twin()];
            color_[e] -= t;
            color_[e->twin()] -= t;

            if (triangulation.is_outer(e) && !out_boundary)
                out_boundary = e;
        }
        if (!out_boundary) return;

        auto flag = halfedge_list_.AddHalfedgeProperty(false);
        std::queue<Halfedge*> q1;
        q1.push(out_boundary);
        flag[out_boundary] = true;

        while (!q1.empty()) {
            Halfedge* e = q1.front();
            q1.pop();
            color_[e] = BLACK;

            if (!flag[e->twin()]) {
                if (color_[e->twin()] == BLACK) {
                    flag[e->twin()] = true;
                    q1.push(e->twin());
                }
            }

            if (!flag[e->next()]) {
                flag[e->next()] = true;
                q1.push(e->next());
            }
        }

        // Step 2. Modify the color for each inner edge.
        std::queue<Halfedge*> q2;
        for (Halfedge* e : halfedge_list_) {
            if (!flag[e]) {
                q2.push(e);
            }
        }

        auto color = halfedge_list_.AddHalfedgeProperty(0);
        for (Halfedge* e : halfedge_list_) {
            color[e] = color_[e];
        }

        while (!q2.empty()) {
            Halfedge* e = q2.front();
            q2.pop();

            int c = color_[e->prev()] | color_[e->next()] | color_[e];
            color_[e] = c;

            if (!flag[e->twin()]) {
                int c1 = color_[e] - color[e];
                if (color_[e->twin()] != (color_[e->twin()] | c1)) {
                    color_[e->twin()] |= c1;
                    q2.push(e->twin());
                }
            }

            if (c != color_[e->next()]) {
                color_[e->next()] = c;
                q2.push(e->next());
            }

            if (c != color_[e->prev()]) {
                color_[e->prev()] = c;
                q2.push(e->prev());
            }
        }

        EraseExtraEdges();
    }

    /**
     * Initialize a triangulation of two polygons and compute the color of
     * edges.
     */
    void Triangulation() {
        if (red_polygon_.empty() && blue_polygon_.empty()) return;

        using Line = typename Arrangement2D<T>::Line;
        Array<Line> lines;
        for (const auto& b : red_polygon_.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    lines.emplace_back(b.polygon.vertex(i),
                                       b.polygon.next_vertex(i),
                                       RED,
                                       0);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    lines.emplace_back(b.polygon.next_vertex(i),
                                       b.polygon.vertex(i),
                                       RED,
                                       0);
                }
            }
        }
        for (const auto& b : blue_polygon_.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    lines.emplace_back(b.polygon.vertex(i),
                                       b.polygon.next_vertex(i),
                                       BLUE,
                                       0);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                   lines.emplace_back(b.polygon.next_vertex(i),
                                      b.polygon.vertex(i),
                                      BLUE,
                                      0);
                }
            }
        }
        arrangement_.set_threshold(threshold_);
        arrangement_.Reset(lines);
        arrangement_.mesh().Clone(&halfedge_list_);
    }

    /**
     * Erase the extra halfedges.
     */
    void EraseExtraEdges() {
        Array<Halfedge*> delete_edges;

        // The edge is useless if the color is the same with its twin's color.
        for (Halfedge* e : halfedge_list_) {
            if (e->id() % 2 == 0 && color_[e] == color_[e->twin()]) {
                delete_edges.push_back(e);
            }
        }
        for (Halfedge* e : delete_edges) {
            halfedge_list_.EraseEdge(e);
        }
    }

    // Threshold for snapping.
    double threshold_;

    // Halfedge list to represent the boolean result.
    Mesh halfedge_list_;

    // Color for each halfedge.
    typename Mesh::template HalfedgeProperty<int> color_;

    // For polygon edges arrangement.
    Arrangement2D<T> arrangement_;

    // Two input polygons.
    MultiPolygon2D<T> red_polygon_, blue_polygon_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_
