//
// Copyright 2015-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_SVG_TERMINAL_H_
#define CODELIBRARY_PLOT_SVG_TERMINAL_H_

#include <fstream>
#include <string>

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/string/string_join.h"
#include "codelibrary/plot/terminal.h"

namespace cl {
namespace plot {

/**
 * SVG terminal
 *
 * Visualizing the figures on the Scalable Array Graphics File.
 */
class SVGTerminal : public Terminal {
public:
    explicit SVGTerminal(int width = 640, int height = 480)
        : Terminal(width, height) {
        Initialize();
    }

    SVGTerminal(const SVGTerminal&) = delete;

    SVGTerminal& operator=(const SVGTerminal&) = delete;

    /**
     * Clear the content of SVG.
     */
    void clear() override {
        content_.clear();
    }

    /**
     * Draw point.
     */
    void DrawPoint(double x, double y) override {
        if (pen_.point_radius <= 0.0) return;

        switch (pen_.point_style) {
        case 'o':
            this->DrawCircle(x, y, pen_.point_radius);
            break;
        case 'r':
            this->DrawRectangle(x - pen_.point_radius,
                                y + pen_.point_radius,
                                pen_.point_radius * 2.0,
                                pen_.point_radius * 2.0);
            break;
        }
    }

    /**
     * Draw circle.
     */
    void DrawCircle(double x, double y, double r) override {
        content_ += ElementStart("circle") +
                    Attribute("cx", x) +
                    Attribute("cy", height_ - y) +
                    Attribute("r", r) +
                    PenAttribute() +
                    EmptyElementEnd();
    }

    /**
     * Draw rectangle.
     */
    void DrawRectangle(double x, double y, double w, double h) override {
        content_ += ElementStart("rect") +
                    PenAttribute() +
                    Attribute("x", x) +
                    Attribute("y", height_ - y) +
                    Attribute("width", w) +
                    Attribute("height", h) +
                    EmptyElementEnd();
    }

    /**
     * Draw line (x1, y1)->(x2, y2).
     */
    void DrawLine(double x1, double y1, double x2, double y2) override {
        content_ += ElementStart("line") +
                    Attribute("x1", x1) +
                    Attribute("y1", height_ - y1) +
                    Attribute("x2", x2) +
                    Attribute("y2", height_ - y2) +
                    LineAttribute() +
                    EmptyElementEnd();
    }

    /**
     * Draw polyline.
     */
    void DrawPolyline(const Array<RPoint2D>& polyline) override {
        std::string points;
        for (const RPoint2D& p : polyline) {
            points += Format("{:g}", p.x) + "," +
                      Format("{:g}", height_ - p.y) + " ";
        }

        content_ += ElementStart("polyline") +
                    Attribute("fill", "none") +
                    LineAttribute() +
                    Attribute("points", points) +
                    EmptyElementEnd();
    }

    /**
     * Draw multiple-polygon.
     */
    void DrawPolygon(const RMultiPolygon2D& polygon) override {
        std::string path;
        for (int i = 0; i < polygon.n_boundaries(); ++i) {
            path += PathAttribute(polygon.boundaries()[i].polygon.vertices());
            path += "Z ";
        }
        path = ElementStart("path") + Attribute("d", path) + EmptyElementEnd();

        content_ += ElementStart("g") +
                    PenAttribute() +
                    Attribute("fill-rule", "evenodd") +
                    + ">\n" +
                    path +
                    ElementEnd("g");
    }

    /**
     * Draw triangle.
     */
    void DrawTriangle(const RPoint2D& p1, const RPoint2D& p2,
                      const RPoint2D& p3) override {
        std::string points;
        points += Format("{:g}", p1.x) + "," +
                  Format("{:g}", height_ - p1.y) + " ";
        points += Format("{:g}", p2.x) + "," +
                  Format("{:g}", height_ - p2.y) + " ";
        points += Format("{:g}", p3.x) + "," +
                  Format("{:g}", height_ - p3.y) + " ";

        content_ += ElementStart("polygon") +
                    PenAttribute() +
                    Attribute("fill-rule", "evenodd") +
                    Attribute("points", points) +
                    EmptyElementEnd();
    }

    /**
     * Draw text at position (x, y).
     */
    void DrawText(double x, double y, const std::string& text) override {
        DrawText(x, y, false, text);
    }

    /**
     * Draw vertical text at position (x, y).
     */
    void DrawVerticalText(double x, double y,
                          const std::string& text) override {
        DrawText(x, y, true, text);
    }

    /**
     * Save to the SVG file.
     */
    void SaveToFile(const std::string& file) const override {
        std::ofstream fout(file);
        fout << head_ << content_ << ElementEnd("svg");
    }

    /**
     * Resize the terminal.
     */
    void Resize(int height, int width) override {
        CHECK(height > 0 && width > 0);

        if (height_ == height && width_ == width) return;

        height_ = height;
        width_  = width;
        Initialize();
    }

private:
    /**
     * Convert a value into XML attribute.
     */
    template <typename T>
    static std::string Attribute(const std::string& attribute_name,
                                 const T& value) {
        return Format("{}=\"{}\" ", attribute_name, value);
    }
    static std::string Attribute(const std::string& attribute_name,
                                 const RGB32Color& color) {
        return Format("{}=\"{}\" ", attribute_name, ColorToString(color));
    }

    /**
     * Return a string that represents starting of a SVG element.
     */
    static std::string ElementStart(const std::string& element_name) {
        return "\t<" + element_name + " ";
    }

    /**
     * Return a string that represents ending of a SVG element.
     */
    static std::string ElementEnd(const std::string& element_name) {
        return "\t</" + element_name + ">\n";
    }

    /**
     * Return a string that represents empty ending of a SVG element.
     */
    static std::string EmptyElementEnd() {
        return "/>\n";
    }

    /**
     * Convert the RGB Color into string.
     */
    static std::string ColorToString(const RGB32Color& c) {
        if (c.alpha() == 255) {
            return Format("rgb({},{},{})", c.red(), c.green(), c.blue());
        }

        return Format("rgba({},{},{},{})", c.red(), c.green(), c.blue(),
                      c.alpha() / 255.0);
    }

    /**
     * Initialize the SVG terminal.
     */
    void Initialize() {
        // Initialize the SVG header.
        head_ = "<?xml " + Attribute("version", "1.0") + "?>\n" +
                "<svg " +
                Attribute("xmlns", "http://www.w3.org/2000/svg") +
                Attribute("xmlns:xlink", "http://www.w3.org/1999/xlink") +
                Attribute("width",  width_) +
                Attribute("height", height_) +
                ">\n\n";
    }

    /**
     * Return pen attribute.
     */
    std::string PenAttribute() const {
        if (pen_.line_width == 0.0) {
            if (!pen_.is_fill)
                return Attribute("fill", "none");

            return Attribute("fill", pen_.fill_color);
        }

        if (!pen_.is_fill) {
            return LineAttribute() + Attribute("fill", "none");
        }

        return LineAttribute() + Attribute("fill", pen_.fill_color);
    }

    /**
     * Get the pen attribute for line drawing.
     */
    std::string LineAttribute() const {
        Array<std::string> list;
        for (int i : pen_.stroke_dash) {
            list.push_back(ToString(i));
        }
        std::string dash = StringJoin(list, ',');
        return Attribute("stroke", pen_.line_color) +
               Attribute("stroke-width", pen_.line_width) +
               Attribute("stroke-dasharray", dash);
    }

    /**
     * DrawText at position(x, y) with the vertical mode.
     */
    void DrawText(double x, double y, bool is_vertical,
                  const std::string& text) {
        y -= font_.size;
        y = height_ - y;

        std::string alignment;
        switch (font_.alignment) {
        case plot::Font::START:
            alignment = "start";
            break;
        case plot::Font::END:
            alignment = "end";
            break;
        case plot::Font::MIDDLE:
            alignment = "middle";
            break;
        }

        std::string font_weight = "normal";

        double offset = font_.size * 0.2;
        std::string rotate = Format("rotate(-90, {:g} {:g})",
                                    x - offset,
                                    y + offset);

        content_ += ElementStart("text") +
                    Attribute("x", x) +
                    Attribute("y", y) +
                    (is_vertical ? Attribute("transform", rotate) : "") +
                    Attribute("fill", font_.color) +
                    Attribute("font-size", font_.size) +
                    Attribute("text-anchor", alignment) +
                    Attribute("font-weight", font_weight) +
                    Attribute("font-family", font_.name) + ">";
        content_ += ElementStart("tspan") +
                    Attribute("x", x) +
                    Attribute("y", y) +
                    ">" +
                    text +
                    ElementEnd("tspan");
        content_ += ElementEnd("text");
    }

    /**
     * Compute a close SVG path element from the given vertices list.
     */
    std::string ClosePath(const Array<RPoint2D>& points) const {
        std::string attribute = PathAttribute(points) + "Z";
        return ElementStart("path") + Attribute("d", attribute) +
               EmptyElementEnd();
    }

    /**
     * Compute a SVG path attribute from the given vertices list.
     */
    std::string PathAttribute(const Array<RPoint2D>& points) const {
        if (points.empty()) return "";

        std::string attribute = Format("M {} {} ", points.front().x,
                                            height_ - points.front().y);
        attribute += "L ";
        for (int i = 1; i < points.size(); ++i) {
            attribute += Format("{} {} ", points[i].x, height_ - points[i].y);
        }

        return attribute;
    }

    std::string head_;    // SVG head.
    std::string content_; // SVG content.
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_SVG_TERMINAL_H_
