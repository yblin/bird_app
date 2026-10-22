//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_TREE_BLOSSOM_GENERATOR_H_
#define CODELIBRARY_WORLD_TREE_BLOSSOM_GENERATOR_H_

#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

class BlossomGenerator {
public:
    static BlossomGenerator* GetInstance() {
        static BlossomGenerator g;
        return &g;
    }

    void CherryBlossom(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.0f, 0.0f, 0.0f},
            {0.33f, 0.45f, 0.45f},
            {0.25f, 0.6f, 0.6f},
            {0.0f, 0.7f, 0.7f},
            {-0.25f, 0.6f, 0.6f},
            {-0.33f, 0.45f, 0.45f},
            {0.49f, 0.42f, 0.6f},
            {0.67f, 0.22f, 0.7f},
            {0.65f, -0.05f, 0.6f},
            {0.53f, -0.17f, 0.45f},
            {0.55f, -0.33f, 0.6f},
            {0.41f, -0.57f, 0.7f},
            {0.15f, -0.63f, 0.6f},
            {0.0f, -0.55f, 0.45f},
            {-0.15f, -0.63f, 0.6f},
            {-0.41f, -0.57f, 0.7f},
            {-0.55f, -0.33f, 0.6f},
            {-0.53f, -0.17f, 0.45f},
            {-0.65f, -0.05f, 0.6f},
            {-0.67f, 0.22f, 0.7f},
            {-0.49f, 0.42f, 0.6f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = {
            0, 1, 2, 0, 2, 3,
            0, 3, 4, 0, 4, 5,
            0, 1, 6, 0, 6, 7,
            0, 7, 8, 0, 8, 9,
            0, 9, 10, 0, 10, 11,
            0, 11, 12, 0, 12, 13,
            0, 13, 14, 0, 14, 15,
            0, 15, 16, 0, 16, 17,
            0, 17, 18, 0, 18, 19,
            0, 19, 20, 0, 20, 5,
        };
    }

    void OrangeBlossom(RenderData* data) const {
        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.0f, 0.0f, 0.0f},
            {-0.055f, 0.165f, 0.11f},
            {-0.125f, 0.56f, 0.365f},
            {0.0f, 0.7f, 0.45f},
            {0.125f, 0.56f, 0.365f},
            {0.055f, 0.165f, 0.11f},
            {0.14f, 0.10f, 0.11f},
            {0.495f, 0.29f, 0.365f},
            {0.665f, 0.215f, 0.45f},
            {0.57f, 0.055f, 0.36f},
            {0.175f, 0.0f, 0.11f},
            {0.14f, -0.1f, 0.11f},
            {0.43f, -0.38f, 0.365f},
            {0.41f, -0.565f, 0.45f},
            {0.23f, -0.53f, 0.365f},
            {0.05f, -0.165f, 0.11f},
            {-0.14f, -0.1f, 0.11f},
            {-0.43f, -0.38f, 0.365f},
            {-0.41f, -0.565f, 0.45f},
            {-0.23f, -0.53f, 0.365f},
            {-0.05f, -0.165f, 0.11f},
            {-0.14f, 0.10f, 0.11f},
            {-0.495f, 0.29, 0.365f},
            {-0.665f, 0.215, 0.45f},
            {-0.57f, 0.055, 0.36f},
            {-0.175f, 0.0f, 0.11f},
            {0.1f, -0.1f, 0.4f},
            {-0.1f, -0.1f, 0.4f},
            {-0.1f, 0.1f, 0.4f},
            {0.1f, 0.1f, 0.4f}
        };

        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = {
            0, 1, 2, 0, 2, 3,
            0, 3, 4, 0, 4, 5,
            0, 6, 7, 0, 7, 8,
            0, 8, 9, 0, 9, 10,
            0, 11, 12, 0, 12, 13,
            0, 13, 14, 0, 14, 15,
            0, 16, 17, 0, 17, 18,
            0, 18, 19, 0, 19, 20,
            0, 21, 22, 0, 22, 23,
            0, 23, 24, 0, 24, 25,
            0, 26, 27,
            0, 27, 28,
            0, 28, 29,
            0, 29, 26,
        };
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_TREE_BLOSSOM_GENERATOR_H_
