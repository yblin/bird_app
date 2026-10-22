//
// Copyright 2020-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_
#define CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_

#include <utility>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/util/color/rgb32_color.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/transform.h"

namespace cl {
namespace world {

/**
 * The data for 3D rendering.
 */
struct RenderData {
    // Type of render data. Should be one of the following value: GL_TRIANGLES,
    // GL_LINES, and GL_POINTS.
    int type;

    // Vertex position.
    Array<FPoint3D> vertices;

    // Color for each vertex.
    Array<RGB32Color> colors;

    // Normal vector for each vertex.
    Array<FVector3D> normals;

    // Texture coordinate for each vertex.
    Array<FPoint2D> texture_coords;

    // Element indices.
    Array<int> indices;

    explicit RenderData(int t = GL_TRIANGLES)
        : type(t) {}

    bool empty() const {
        return vertices.empty();
    }

    void clear() {
        Array<FPoint3D>().swap(vertices);
        Array<RGB32Color>().swap(colors);
        Array<FVector3D>().swap(normals);
        Array<FPoint2D>().swap(texture_coords);
        Array<int>().swap(indices);
    }

    /**
     * Swap data to reduce memory copy.
     */
    void Swap(RenderData* data) {
        CHECK(data);

        std::swap(data->type, type);
        vertices.swap(data->vertices);
        colors.swap(data->colors);
        normals.swap(data->normals);
        texture_coords.swap(data->texture_coords);
        indices.swap(data->indices);
    }

    /**
     * Transform the render data.
     */
    void Transform(const gl::FTransform& transform) {
        for (auto& v : vertices) {
            v = transform(v);
        }
        for (auto& v : normals) {
            v = transform(v);
        }
    }

    /**
     * Translate the render data.
     */
    void Translate(const FVector3D& v) {
        for (auto& v1 : vertices) {
            v1 += v;
        }
    }

    /**
     * Return the bounding box of render data.
     */
    FBox3D GetBoundingBox() const {
        Array<FPoint3D> points;
        points.reserve(indices.size());
        for (int i : indices) {
            points.push_back(vertices.at(i));
        }
        return FBox3D(vertices.begin(), vertices.end());
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_
