//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/boolean/polygon_union_2d.h"

namespace cl {
namespace test {

TEST(PolygonUnion2DTest, Test) {
    RBox2D box1(0.0, 1.0, 0.0, 1.0);
    RBox2D box2(1.0, 2.0, 0.0, 1.0);
    RPolygon2D poly1(box1), poly2(box2);
    Array<RMultiPolygon2D> polygons;
    polygons.emplace_back(poly1);
    polygons.emplace_back(poly2);

    geometry::PolygonUnion2D<double> polygon_union;
    RMultiPolygon2D res;
    polygon_union(polygons, 1e-2, &res);

    ASSERT(res.n_boundaries() == 1);
    auto poly = res.boundaries().front().polygon;
    ASSERT_EQ(poly[0], RPoint2D(2.0, 1.0));
    ASSERT_EQ(poly[1], RPoint2D(1.0, 1.0));
    ASSERT_EQ(poly[2], RPoint2D(0.0, 1.0));
    ASSERT_EQ(poly[3], RPoint2D(0.0, 0.0));
    ASSERT_EQ(poly[4], RPoint2D(1.0, 0.0));
    ASSERT_EQ(poly[5], RPoint2D(2.0, 0.0));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_TEST_H_
