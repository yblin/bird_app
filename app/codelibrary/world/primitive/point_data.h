//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_POINT_DATA_H_
#define CODELIBRARY_WORLD_PRIMITIVE_POINT_DATA_H_

#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Points render data.
 */
struct PointData : public RenderData {
    PointData() : RenderData(GL_POINTS) {}

    explicit PointData(const Array<FPoint3D>& points)
        : RenderData(GL_POINTS) {
        vertices = points;
        indices.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            indices[i] = i;
        }
    }

    PointData(const Array<FPoint3D>& points, const Array<RGB32Color>& cs)
        : RenderData(GL_POINTS) {
        vertices = points;
        colors = cs;
        indices.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            indices[i] = i;
        }
    }

    PointData(const Array<FPoint3D>& points, const RGB32Color& c)
        : RenderData(GL_POINTS) {
        vertices = points;
        colors.assign(vertices.size(), c);
        indices.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            indices[i] = i;
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_POINT_DATA_H_
