//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_CONE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_CONE_H_

#include "codelibrary/base/numbers.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Solid cone.
 */
struct Cone : public RenderData {
    /**
     * Construct a unit cylinder.
     */
    explicit Cone(int n_slices = 32)
        : RenderData(GL_TRIANGLES) {
        CHECK(n_slices >= 3);

        float angle = 2.0f * numbers::pi_v<float> / n_slices;
        float r = 0.0f;
        for (int i = 0; i <= n_slices; ++i, r += angle) {
            vertices.emplace_back(std::cos(r), std::sin(r), 0.0f);
        }
        vertices.emplace_back(0.0f, 0.0f, 0.0f);
        vertices.emplace_back(0.0f, 0.0f, 1.0f);

        int bottom = vertices.size() - 2;
        int top = vertices.size() - 1;

        normals.resize(vertices.size());
        texture_coords.resize(vertices.size());

        // Add cone surface.
        for (int i = 0; i <= n_slices; ++i) {
            FVector3D v = vertices[i].ToVector();
            normals[i]  = v;

            texture_coords[i] = FPoint2D(0.5f * (v.x + 1.0f),
                                         0.5f * (v.y + 1.0f));

            if (i != n_slices) {
                indices.push_back(i);
                indices.push_back(i + 1);
                indices.push_back(top);

                indices.push_back(i + 1);
                indices.push_back(i);
                indices.push_back(bottom);
            }
        }
        normals[bottom] = FVector3D(0.0f, 0.0f, -1.0f);
        normals[top] = FVector3D(0.0f, 0.0f, 1.0f);

        texture_coords[bottom] = FPoint2D(0.5f, 0.5f);
        texture_coords[top] = FPoint2D(0.5f, 0.5f);
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_CONE_H_
