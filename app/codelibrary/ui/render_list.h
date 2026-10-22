//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_RENDER_LIST_H_
#define CODELIBRARY_UI_RENDER_LIST_H_

#include <algorithm>

#include "third_party/imgui/imgui.h"

#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/mesh/polygon_triangulation_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace ui {

/**
 * Render geometric objects by ImDrawList.
 */
class RenderList {
public:
    RenderList() = default;

    explicit RenderList(ImDrawList* draw_list, int h)
      : draw_list_(draw_list), height_(h) {
        CHECK(draw_list);
        CHECK(h >= 0);
    }

    /**
     * Render a circle to the current ImGui context.
     */
    template <typename T>
    void RenderCircle(const Circle2D<T>& circle, const RGB32Color& color,
                      int n_segments = 0, float thickness = 1.0f) {
        CHECK(draw_list_);

        draw_list_->AddCircle(ToImVec2(circle.center()),
                              static_cast<float>(circle.radius()),
                              color.ToUInt(),
                              n_segments,
                              thickness);
    }

    /**
     * Render a filled circle.
     */
    template <typename T>
    void RenderCircleFilled(const Circle2D<T>& circle, const RGB32Color& color,
                            int n_segments = 0) {
        CHECK(draw_list_);

        draw_list_->AddCircleFilled(ToImVec2(circle.center()),
                                    static_cast<float>(circle.radius()),
                                    color.ToUInt(),
                                    n_segments);
    }

    /**
     * Render a line.
     */
    template <typename T>
    void RenderLine(const Point2D<T>& p1, const Point2D<T>& p2,
                    const RGB32Color& color, float thickness = 1.0f) {
        CHECK(draw_list_);

        draw_list_->AddLine(ToImVec2(p1), ToImVec2(p2), color.ToUInt(),
                            thickness);
    }

    /**
     * Render a dash line.
     */
    template <typename T>
    void RenderDashLine(const Point2D<T>& p1, const Point2D<T>& p2,
                        const RGB32Color& color, float dash_length,
                        float thickness = 1.0f) {
        CHECK(draw_list_);

        ImVec2 q1 = ToImVec2(p1), q2 = ToImVec2(p2);
        const ImVec2 dir = ImVec2(q2.x - q1.x, q2.y - q1.y);
        const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        const ImVec2 normalized_dir = ImVec2(dir.x / length, dir.y / length);

        for (float d = 0.0f; d < length; d += dash_length * 2.0f) {
            float d1 = std::min(d + dash_length, length);

            draw_list_->AddLine(ImVec2(q1.x + d * normalized_dir.x,
                                       q1.y + d * normalized_dir.y),
                                ImVec2(q1.x + d1 * normalized_dir.x,
                                       q1.y + d1 * normalized_dir.y),
                                color.ToUInt(),
                                thickness);
        }
    }

    /**
     * Render a line.
     */
    template <typename T>
    void RenderLine(const Segment2D<T>& seg, const RGB32Color& color,
                    float thickness = 1.0f) {
        RenderLine(seg.lower_point(), seg.upper_point(), color, thickness);
    }

    /**
     * Render a polyline.
     */
    template <typename T>
    void RenderPolyLine(const Polyline<Point2D<T>>& polyline,
                        const RGB32Color& color,
                        float thickness = 1.0f) {
        CHECK(draw_list_);

        for (int i = 0; i + 1 < polyline.size(); ++i) {
            RenderLine(polyline[i], polyline[i + 1], color, thickness);
        }
    }

    /**
     * Render a box.
     */
    template <typename T>
    void RenderBox(const Box2D<T>& box, const RGB32Color& color,
                   float thickness = 1.0f) {
        CHECK(draw_list_);

        draw_list_->AddRect(ToImVec2(box.x_min(), box.y_min()),
                            ToImVec2(box.x_max(), box.y_max()),
                            color.ToUInt(),
                            0.0f,
                            0,
                            thickness);
    }

    /**
     * Render a filled box.
     */
    template <typename T>
    void RenderBoxFilled(const Box2D<T>& box, const RGB32Color& color) {
        CHECK(draw_list_);

        draw_list_->AddRectFilled(ToImVec2(box.x_min(), box.y_min()),
                                  ToImVec2(box.x_max(), box.y_max()),
                                  color.ToUInt(),
                                  0.0f,
                                  0);
    }

    /**
     * Render a triangle (best organized in clockwise).
     */
    template <typename T>
    void RenderTriangle(const Point2D<T>& p1, const Point2D<T>& p2,
                        const Point2D<T>& p3, const RGB32Color& color,
                        float thickness = 1.0f) {
        CHECK(draw_list_);

        draw_list_->AddTriangle(ToImVec2(p1), ToImVec2(p2), ToImVec2(p3),
                                color.ToUInt(), thickness);
    }
    template <typename T>
    void RenderTriangle(const Triangle2D<T>& triangle,
                        const RGB32Color& color,
                        float thickness = 1.0f) {
        CHECK(!triangle.empty());

        RenderTriangle(triangle.vertices()[0], triangle.vertices()[1],
                       triangle.vertices()[2], color,
                       thickness);
    }

    /**
     * Render a filled triangle.
     */
    template <typename T>
    void RenderTriangleFilled(const Point2D<T>& p1, const Point2D<T>& p2,
                              const Point2D<T>& p3, const RGB32Color& color) {
        CHECK(draw_list_);

        draw_list_->AddTriangleFilled(ToImVec2(p1), ToImVec2(p2), ToImVec2(p3),
                                      color.ToUInt());
    }
    template <typename T>
    void RenderTriangleFilled(const Triangle2D<T>& triangle,
                              const RGB32Color& color) {
        CHECK(!triangle.empty());

        RenderTriangleFilled(triangle.vertices()[0], triangle.vertices()[1],
                             triangle.vertices()[2], color);
    }

    /**
     * Render a quad (best organized in countclockwise).
     */
    template <typename T>
    void RenderQuad(const Array<Point2D<T>>& vertices, const RGB32Color& color,
                    float thickness = 1.0f) {
        CHECK(draw_list_);
        CHECK(vertices.size() == 4);

        draw_list_->AddQuad(ToImVec2(vertices[0]), ToImVec2(vertices[1]),
                            ToImVec2(vertices[2]), ToImVec2(vertices[3]),
                            color.ToUInt(),
                            thickness);
    }

    /**
     * Render a filed quad (best organized in countclockwise).
     */
    template <typename T>
    void RenderQuadFilled(const Array<Point2D<T>>& vertices,
                          const RGB32Color& color) {
        CHECK(draw_list_);
        CHECK(vertices.size() == 4);

        draw_list_->AddQuadFilled(ToImVec2(vertices[0]), ToImVec2(vertices[1]),
                                  ToImVec2(vertices[2]), ToImVec2(vertices[3]),
                                  color.ToUInt());
    }

    /**
     * Render a polygon.
     */
    template <typename T>
    void RenderPolygon(const Array<Point2D<T>>& points,
                       const RGB32Color& color,
                       float thickness = 1.0f) {
        for (int i = 0; i < points.size(); ++i) {
            int next = i + 1 == points.size() ? 0 : i + 1;
            RenderLine(points[i], points[next], color, thickness);
        }
    }

    /**
     * Render a polygon.
     */
    template <typename T>
    void RenderPolygon(const Polygon2D<T>& polygon, const RGB32Color& color,
                       float thickness = 1.0f) {
        for (int i = 0; i < polygon.size(); ++i) {
            RenderLine(polygon[i], polygon.next_vertex(i), color, thickness);
        }
    }

    /**
     * Render a multi-polygon.
     */
    template <typename T>
    void RenderPolygon(const MultiPolygon2D<T>& polygon,
                       const RGB32Color& color,
                       float thickness = 1.0f) {
        for (const auto& b : polygon.boundaries()) {
            RenderPolygon(b.polygon, color, thickness);
        }
    }

    /**
     * Render a multi-polygon (in the form of triangulation).
     */
    template <typename T>
    void RenderPolygonFilled(const geometry::PolygonTriangulation2D<T>&
                             triangulation,
                             const RGB32Color& color) {
        CHECK(draw_list_);

        draw_list_->Flags &= ~ImDrawListFlags_AntiAliasedFill;

        Array<Triangle2D<T>> triangles;
        triangulation.ToTriangles(&triangles);
        for (const Triangle2D<T>& triangle : triangles) {
            RenderTriangleFilled(triangle, color);
        }
        draw_list_->Flags |= ImDrawListFlags_AntiAliasedFill;
    }

    /**
     * Render text at 'p'.
     */
    template <typename T>
    void RenderText(const std::string& text, const Point2D<T>& p,
                    const RGB32Color& color) {
        CHECK(draw_list_);

        draw_list_->AddText(ToImVec2(p), color.ToUInt(), text.c_str());
    }

protected:
    /**
     * Map 2D point to ImVec2.
     */
    template <typename T>
    ImVec2 ToImVec2(const Point2D<T>& p) const {
        return ToImVec2(p.x, p.y);
    }
    template <typename T>
    ImVec2 ToImVec2(T x, T y) const {
        return ImVec2(static_cast<float>(x), static_cast<float>(height_ - y));
    }

    // Draw list of ImGui.
    ImDrawList* draw_list_ = nullptr;

    // The height of the UI window.
    int height_ = 0;
};

} // namespace ui
} // namespace cl

#endif // CODELIBRARY_UI_RENDER_LIST_H_
