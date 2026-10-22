//
// Copyright 2014-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_

#include <random>

#include "codelibrary/base/numbers.h"
#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/mesh/delaunay_2d.h"

namespace cl {
namespace test {

TEST(Delaunay2DTest, EmptyInputPoints) {
    Array<RPoint2D> points;
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, OneInputPoint) {
    Array<RPoint2D> points = {{0.0, 0.0}};
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, TwoInputPoints) {
    Array<RPoint2D> points = { {1.0, 1.0}, {0.0, 1.0} };
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, FourInputPoints) {
    Array<RPoint2D> points = { {-24304000.0, -858951.0},
                               {-904000.0,   -858951.0},
                               {-904000.0,   11041049.0},
                               {-24304000.0, 11041049.0} };
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, CollinearInputPoints) {
    Array<RPoint2D> points(100);
    for (int i = 0; i < points.size(); ++i) {
        points[i].x = points[i].y = static_cast<double>(i);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, UniformGridInputPoints) {
    Array<RPoint2D> points(100);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            points[i * 10 + j].x = i;
            points[i * 10 + j].y = j;
        }
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, ConcyclicInputPoints) {
    Array<RPoint2D> points(360);
    for (int i = 0; i < 360; ++i) {
        points[i].x = std::cos(i * numbers::pi / 180.0);
        points[i].y = std::sin(i * numbers::pi / 180.0);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, RandomInputPoints) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100);
    for (int i = 0; i < 100; ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(delaunay.IsValid());
}

TEST(Delaunay2DTest, RandomInsert) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    geometry::Delaunay2D<double> delaunay;
    for (int i = 0; i < 100; ++i) {
        double x = uniform(random);
        double y = uniform(random);
        delaunay.Insert(RPoint2D(x, y));
        ASSERT(delaunay.IsValid());
    }
}


TEST(Delaunay2DTest, RandomEraseVertices) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100);
    for (int i = 0; i < points.size(); ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);

    Array<int> seq(points.size());
    for (int i = 0; i < points.size(); ++i) {
        seq[i] = i;
    }
    std::shuffle(seq.begin(), seq.end(), random);
    for (int i : seq) {
        delaunay.Erase(points[i]);
        ASSERT(delaunay.IsValid());
    }
}

TEST(Delaunay2DTest, Performance100000) {
    std::mt19937 random;

    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100000);
    for (int i = 0; i < 100000; ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_
