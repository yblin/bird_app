//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_POLYGON_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_POLYGON_2D_H_

#include <unordered_set>

#include "codelibrary/geometry/topology/even_odd_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Winding rules.
 */
enum class WindingRule {
    EVEN_ODD,
    NON_ZERO,
    POSITIVE,
    NEGATIVE,
};

/**
 * Extract valid polygons from a topological arrangement based on specified
 * winding number rule.
 */
template <typename T>
void WindingPolygon2D(const Arrangement2D<T>& arrangement,
                      const Array<int>& winding,
                      WindingRule rule,
                      MultiPolygon2D<T>* result) {
    CHECK(arrangement.regions().size() == winding.size() ||
          arrangement.regions().size() + 1 == winding.size());

    using Halfedge = typename Arrangement2D<T>::Halfedge;

    Arrangement2D<T> arrangement1;
    std::unordered_set<Halfedge*> hash;

    MultiPolygon2D<T> region;
    Array<Array<Halfedge*>> edge_lists;
    for (int i = 0; i < arrangement.regions().size(); ++i) {
        bool is_target = false;
        switch (rule) {
        case WindingRule::EVEN_ODD:
            is_target = winding[i] % 2 == 0;
            break;
        case WindingRule::NON_ZERO:
            is_target = winding[i] >= 0;
            break;
        case WindingRule::POSITIVE:
            is_target = winding[i] > 0;
            break;
        case WindingRule::NEGATIVE:
            is_target = winding[i] < 0;
            break;
        }

        if (is_target) {
            arrangement.GetRegion(i, &edge_lists);
            for (auto& edges : edge_lists) {
                for (auto e : edges) {
                    hash.insert(e);
                }
            }
        }
    }
    for (auto e : hash) {
        if (hash.find(e->twin()) == hash.end()) {
            arrangement1.InsertWithoutCross(e->source_point(),
                                            e->target_point());
        }
    }
    arrangement1.Arrange();
    EvenOddPolygon2D(arrangement1, result);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_POLYGON_2D_H_
