//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_CIRCLE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_CIRCLE_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/geometry/circle_3d.h"
#include "codelibrary/geometry/transform_3d.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * A planar circle.
 */
struct Circle : public RenderData {
    /**
     * Construct a unit circle.
     */
    explicit Circle(int n_slices = 32)
        : RenderData(GL_TRIANGLES) {
        CHECK(n_slices >= 3);

        float angle = 2.0f * numbers::pi_v<float> / n_slices;
        float r = 0.0f;
        for (int i = 0; i <= n_slices; ++i, r += angle) {
            vertices.emplace_back(std::cos(r), std::sin(r), 0.0f);
            texture_coords.emplace_back((vertices.back().x + 1.0f) * 0.5f,
                                        (vertices.back().y + 1.0f) * 0.5f);
        }
        vertices.emplace_back(0.0f, 0.0f, 0.0f);
        texture_coords.emplace_back(0.5f, 0.5f);
        normals.resize(vertices.size(), FVector3D(0.0f, 0.0f, 1.0f));

        for (int i = 0; i < vertices.size() - 1; ++i) {
            int next = i + 1 == vertices.size() - 1 ? 0 : i + 1;
            indices.push_back(vertices.size() - 1);
            indices.push_back(i);
            indices.push_back(next);
        }
    }

    explicit Circle(const FCircle3D& circle, int n_slices = 32) {
        CHECK(n_slices >= 3);

        float angle = 2.0f * numbers::pi_v<float> / n_slices;
        float r = 0.0f;
        FQuaternion rot(FVector3D(0.0, 0.0, 1.0), circle.normal());
        FVector3D v = circle.center().ToVector();
        for (int i = 0; i <= n_slices; ++i, r += angle) {
            FPoint3D p(std::cos(r) * circle.radius(),
                       std::sin(r) * circle.radius(),
                       0.0f);
            texture_coords.emplace_back((p.x + 1.0f) * 0.5f,
                                        (p.y + 1.0f) * 0.5f);
            p = geometry::Rotate(p, rot);
            p += v;
            vertices.push_back(p);
        }
        vertices.push_back(circle.center());
        texture_coords.emplace_back(0.5f, 0.5f);
        normals.resize(vertices.size(), circle.normal());

        for (int i = 0; i < n_slices; ++i) {
            indices.push_back(vertices.size() - 1);
            indices.push_back(i);
            indices.push_back(i + 1);
        }
    }

    Circle(const FPoint3D& center, const FVector3D& normal, float radius,
           int n_slices = 32)
        : Circle(FCircle3D(center, normal, radius), n_slices) {}
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_CIRCLE_H_
