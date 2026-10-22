//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/topology/polygon_topology_fix_2d.h"

namespace cl {
namespace test {

TEST(PolygonTopologyFix2DTest, FixPolygonWithHoles) {
    Array<RPoint2D> points1 = {
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}
    };
    RPolygon2D polygon1(points1);
    Array<RPoint2D> points2 = {
        {5.0, 10.0}, {7.0, 10.0}, {7.0, 6.0}, {5.0, 6.0}
    };
    RPolygon2D polygon2(points2);
    RMultiPolygon2D input1;
    input1.Insert(polygon1, true);
    input1.Insert(polygon2, false);

    Array<RMultiPolygon2D> output1;
    geometry::FixPolygonWithHoles(input1, &output1);
    ASSERT_EQ(output1.size(), 1);
    ASSERT_EQ(output1[0].boundaries().size(), 1);

    RMultiPolygon2D input2;
    input2.Insert(output1[0].boundaries()[0].polygon, true);
    input2.Insert(polygon2, false);
    Array<RMultiPolygon2D> output2;
    geometry::FixPolygonWithHoles(input2, &output2);
    ASSERT_EQ(output2.size(), 1);
    ASSERT_EQ(output2[0].boundaries().size(), 1);

    Array<RPoint2D> points3 = {
        {5.0, -10.0}, {5.0, 20.0}, {7.0, 20.0}, {7.0, -10.0}
    };
    RPolygon2D polygon3(points3);
    RMultiPolygon2D input3;
    input3.Insert(polygon1, true);
    input3.Insert(polygon3, false);
    Array<RMultiPolygon2D> output3;
    geometry::FixPolygonWithHoles(input3, &output3);
    ASSERT_EQ(output3.size(), 2);
    ASSERT_EQ(output3[0].boundaries().size(), 1);
    ASSERT_EQ(output3[1].boundaries().size(), 1);

    RMultiPolygon2D input4;
    input3.Insert(polygon1, true);
    input3.Insert(polygon1, false);
    Array<RMultiPolygon2D> output4;
    geometry::FixPolygonWithHoles(input4, &output4);
    ASSERT_EQ(output4.size(), 0);


//    for (auto o : output4) {
//        for (auto b : o) {
//            LOG(ERROR) << b.polygon.vertices();
//            LOG(ERROR) << b.is_outer;
//        }
//    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_TOPOLOGY_POLYGON_TOPOLOGY_FIX_2D_TEST_H_
