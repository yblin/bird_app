//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_LINE_DATA_H_
#define CODELIBRARY_WORLD_PRIMITIVE_LINE_DATA_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/circle_3d.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/transform_3d.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Lines render data.
 */
struct LineData : public RenderData {
    LineData() : RenderData(GL_LINES) {}

    LineData(const FPoint3D& p1, const FPoint3D& p2)
        : RenderData(GL_LINES) {
        vertices = {p1, p2};
        indices = {0, 1};
    }

    LineData(const FPoint3D& p1, const FPoint3D& p2, const RGB32Color& c)
        : LineData(p1, p2) {
        colors = {c, c};
    }

    /**
     * Construct a line set of box's contour.
     */
    explicit LineData(const FBox3D& box)
        : RenderData(GL_LINES) {
        vertices = {
            {box.x_min(), box.y_min(), box.z_min()},
            {box.x_min(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_min(), box.z_min()},

            {box.x_min(), box.y_min(), box.z_max()},
            {box.x_min(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_min(), box.z_max()}
        };
        indices = {0, 1, 1, 2, 2, 3, 3, 0,
                   4, 5, 5, 6, 6, 7, 7, 4,
                   0, 4, 1, 5, 2, 6, 3, 7};
    }
    LineData(const FBox3D& box, const RGB32Color& c)
        : LineData(box) {
        colors.assign(8, c);
    }

    explicit LineData(const FPolyline3D& polyline)
        : RenderData(GL_LINES) {
        if (polyline.size() < 2) return;

        vertices = polyline;
        for (int i = 0; i + 1 < polyline.size(); ++i) {
            indices.push_back(i);
            indices.push_back(i + 1);
        }
    }

    LineData(const FPolyline3D& polyline, const RGB32Color& c)
        : LineData(polyline) {
        colors.assign(vertices.size(), c);
    }

    explicit LineData(const Array<FPoint3D>& polyline)
        : LineData(FPolyline3D(polyline)) {}

    LineData(const Array<FPoint3D>& polyline, const RGB32Color& c)
        : LineData(polyline) {
        colors.assign(vertices.size(), c);
    }

    explicit LineData(const FCircle3D& circle, int n_slices = 32)
        : RenderData(GL_LINES) {
        CHECK(n_slices >= 3);

        float angle = 2.0f * numbers::pi_v<float> / n_slices;
        float r = 0.0f;
        FQuaternion rot(FVector3D(0.0f, 0.0f, 1.0f), circle.normal());
        FVector3D v = circle.center().ToVector();
        for (int i = 0; i <= n_slices; ++i, r += angle) {
            FPoint3D p(std::cos(r) * circle.radius(),
                       std::sin(r) * circle.radius(),
                       0.0f);
            p = geometry::Rotate(p, rot);
            p += v;
            vertices.push_back(p);
        }

        for (int i = 0; i + 1 < vertices.size(); ++i) {
            indices.push_back(i);
            indices.push_back(i + 1);
        }
    }

    LineData(const FCircle3D& circle, const RGB32Color& c, int n_slices = 32)
        : LineData(circle, n_slices) {
        colors.assign(vertices.size(), c);
    }

    /**
     * Lines from surface mesh faces.
     */
    explicit LineData(const geometry::SurfaceMesh<FPoint3D>& mesh)
        : RenderData(GL_LINES) {
        vertices.resize(mesh.n_allocated_vertices());
        for (auto v : mesh.vertices()) {
            vertices[v->id()] = v->point();
        }
        for (auto e : mesh.edges()) {
            if (e->id() % 2 == 1) continue;
            indices.push_back(e->source()->id());
            indices.push_back(e->target()->id());
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_LINE_DATA_H_
