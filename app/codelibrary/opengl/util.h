//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_UTIL_H_
#define CODELIBRARY_OPENGL_UTIL_H_

#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace gl {

/**
 * glClearColor().
 */
inline void ClearColor(const RGB32Color& color) {
    glClearColor(color.red()   / 255.0f, color.green() / 255.0f,
                 color.blue()  / 255.0f,
                 color.alpha() / 255.0f);
}

/**
 * glViewport().
 */
template <typename T>
void SetViewport(const Box2D<T>& viewport) {
    glViewport(static_cast<int>(viewport.x_min()),
               static_cast<int>(viewport.y_min()),
               static_cast<GLsizei>(viewport.x_length()),
               static_cast<GLsizei>(viewport.y_length()));
}

/**
 * glScissor().
 */
template <typename T>
void SetScissor(const Box2D<T>& viewport) {
    glScissor(static_cast<GLint>(viewport.x_min()),
              static_cast<GLint>(viewport.y_min()),
              static_cast<GLsizei>(viewport.x_length()),
              static_cast<GLsizei>(viewport.y_length()));
}

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_UTIL_H_
