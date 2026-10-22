//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_GROUND_H_
#define CODELIBRARY_WORLD_GROUND_H_

#include "codelibrary/world/node.h"
#include "codelibrary/world/primitive/line_data.h"

namespace cl {
namespace world {

/**
 * Ground show as grid lines.
 */
class Ground : public Node {
public:
    Ground() : Node("Ground") {
        Initialize();
    }

    explicit Ground(const RGB32Color& grid_color1,
                    const RGB32Color& grid_color2)
        : Node("Ground"),
          grid_color_level1_(grid_color1),
          grid_color_level2_(grid_color2) {
        Initialize();
    }

private:
    /**
     * Initialize ground.
     */
    void Initialize() {
        render_mode_ = RenderMode::USE_OWN_SHADER;
        cast_shadow_ = false;
        receive_shadow_ = false;
        this->AddRenderObject(&grid_lines_);
        ResetGridLines();
    }

    /**
     * Reset grid lines.
     */
    void ResetGridLines() {
        grid_lines_.clear();

        float x_gap = 2.0f / n_grid_lines_;
        float y_gap = 2.0f / n_grid_lines_;

        for (int i = 0; i <= n_grid_lines_; ++i) {
            float x = -1.0f + x_gap * i;
            float y = -1.0f + y_gap * i;

            if (i % 10 == 0) {
                grid_lines_.Add(LineData(FPoint3D(x, -1.0f, 0.0f),
                                         FPoint3D(x,  1.0f, 0.0f),
                                         grid_color_level1_));
                grid_lines_.Add(LineData(FPoint3D(-1.0f, y, 0.0f),
                                         FPoint3D(1.0f, y, 0.0f),
                                         grid_color_level1_));
            } else {
                grid_lines_.Add(LineData(FPoint3D(x, -1.0f, 0.0f),
                                         FPoint3D(x,  1.0f, 0.0f),
                                         grid_color_level2_));
                grid_lines_.Add(LineData(FPoint3D(-1.0f, y, 0.0f),
                                         FPoint3D(1.0f, y, 0.0f),
                                         grid_color_level2_));
            }
        }
    }

    // Color of grid lines.
    RGB32Color grid_color_level1_ = RGB32Color(32, 32, 32);
    RGB32Color grid_color_level2_ = RGB32Color(64, 64, 64);

    // The number of grid lines in one dimension.
    const int n_grid_lines_ = 100;

    // Render object.
    RenderObject grid_lines_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_GROUND_H_
