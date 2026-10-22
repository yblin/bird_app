//
// Copyright 2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_TEXT_H_
#define CODELIBRARY_PLOT_TEXT_H_

#include <string>

#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/plot/font.h"

namespace cl {
namespace plot {

/**
 * Text for plotting.
 */
struct Text {
    friend class BasePlot;

    Text() = default;

    Text(const RPoint2D& position, const std::string& text, double font_size)
        : position_(position), data_(text), font_size_(font_size) {}

    Text& alignment(Font::Alignment alignment) {
        alignment_ = alignment;
        return *this;
    }

    Text& color(const RGB32Color& color) {
        color_ = color;
        return *this;
    }

    Text& color(int r, int g, int b, int a = 255) {
        color_ = RGB32Color(r, g, b, a);
        return *this;
    }

    Text& font_size(double font_size) {
        font_size_ = font_size;
        return *this;
    }

protected:
    RGB32Color color_ = RGB32Color::Black();
    Font::Alignment alignment_ = Font::MIDDLE;
    RPoint2D position_;
    std::string data_;
    double font_size_ = 10.0;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_TEXT_H_
