//
// Copyright 2022-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_UI_H_
#define CODELIBRARY_UI_UI_H_

#include <string>

#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/ui/icons_font_awesome.h"
#include "codelibrary/ui/renderable.h"
#include "codelibrary/ui/render_list.h"
#include "codelibrary/util/io/file_util.h"

namespace cl {

/**
 * Graphical User interface for Codelibrary.
 *
 * The following code is adapted from ImGUI library.
 *
 * Note that the following code is thread unsafe.
 *
 * ----------------------------------------------------------------
 * Sample usage:
 *
 * cl::Window window("Codelibrary UI example");
 * cl::UI ui(&window);
 *
 * while (!window.IsClosed()) {
 *      window.PollEvents();
 *
 *      // Clear GL color bits.
 *      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
 *      glClear(GL_COLOR_BUFFER_BIT);
 *
 *      ui.NewFrame();
 *      ui.Begin("Info");
 *      ImGui::Button("It is a button");
 *      ImGui::Separator();
 *      ImGui::Text("It is a text");
 *      ui.End();
 *      ui.Render();
 *      window.SwapBuffers();
 * }
 * ----------------------------------------------------------------
 */
class UI : public ui::Renderable {
public:
    /**
     * Used to set the position of next window.
     */
    enum WindowPos {
        LEFT_WINDOW,
        BOTTOM_WINDOW,
        TOP_WINDOW,
        LEFT_BOTTOM_WINDOW,
        LEFT_TOP_WINDOW,
        RIGHT_BOTTOM_WINDOW,
        RIGHT_TOP_WINDOW
    };

    UI() = delete;

    explicit UI(Window* window)
        : ui::Renderable(window) {
        this->ResetToDefaultFont(window->width() / 1500 * 13.0f);
    }

    UI(const UI&) = delete;

    UI& operator=(const UI&) = delete;

    /**
     * Clear all existing fonts and reset the current font to default font.
     *
     * Parameter:
     *  pixel_size - pixel size of the default font, the default value is 13.
     */
    void ResetToDefaultFont(float pixel_size = 13.0f) {
        CHECK(pixel_size > 0.0f);

        font_size_ = pixel_size;
        IO().Fonts->Clear();
        ImFontConfig icons_config;
        icons_config.SizePixels = pixel_size;
        IO().Fonts->AddFontDefault(&icons_config);
    }

    /**
     * Clear all fonts and reset the current font to a chinese font.
     */
    void ResetToChineseFont(const std::string& name) {
        IO().Fonts->Clear();
        IO().Fonts->AddFontFromFileTTF(name.c_str(), font_size_, nullptr,
                                       IO().Fonts->GetGlyphRangesChineseFull());
    }

    /**
     * Append Font-Awesome icons font to ImGui font set.
     *
     * Parameter:
     *  name       - font-awesome file path.
     *  pixel_size - pixel size of the icon font.
     */
    void AddFontAwesomeIcons(const std::string& name, float pixel_size) {
        CHECK(pixel_size > 0.0f);
        CHECK(file_util::IsFileExist(name))
            << "File: '" << name << "' is not exist.";

        // FontAwesome fonts need to have their sizes reduced by 2.0f/3.0f in
        // order to align correctly.
        float icon_font_size = pixel_size * 2.0f / 3.0f;

        if (IO().Fonts->Fonts.empty()) {
            this->ResetToDefaultFont(pixel_size);
        }

        // icons_ranges should be static here. Otherwise, AddFontFromFileTTF()
        // doesn't do what we want.
        static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};

        ImFontConfig icons_config;
        icons_config.MergeMode = true;
        icons_config.PixelSnapH = true;
        icons_config.GlyphMinAdvanceX = icon_font_size;

        IO().Fonts->AddFontFromFileTTF(name.c_str(), icon_font_size,
                                       &icons_config, icons_ranges);
    }
    void AddFontAwesomeIcons(const std::string& name) {
        return AddFontAwesomeIcons(name, font_size_);
    }

    /**
     * Equal to ImGui::Begin().
     */
    bool Begin(const char* name, bool* p_open = nullptr,
               ImGuiWindowFlags flags = 0) {
        previous_imgui_context_ = ImGui::GetCurrentContext();
        ImGui::SetCurrentContext(imgui_context_);
        return ImGui::Begin(name, p_open, flags);
    }

    /**
     * Creating a simple static window without decoration.
     */
    bool BeginStaticWindow(const char* name, WindowPos pos, float gap = 10) {
        this->SetNextWindowPos(pos, 0, gap);
        return Begin(name, nullptr, ImGuiWindowFlags_NoDecoration       |
                                    ImGuiWindowFlags_AlwaysAutoResize   |
                                    ImGuiWindowFlags_NoSavedSettings    |
                                    ImGuiWindowFlags_NoFocusOnAppearing |
                                    ImGuiWindowFlags_NoNav);
    }

    /**
     * Equal to ImGui::End().
     */
    void End() {
        ImGui::End();
        ImGui::SetCurrentContext(previous_imgui_context_);
    }

    /**
     * Equal to ImGui::SetNextWindowPos().
     */
    void SetNextWindowPos(const ImVec2& pos, ImGuiCond cond = 0,
                          const ImVec2& pivot = ImVec2(0, 0)) {
        ContextControl control(imgui_context_);
        return ImGui::SetNextWindowPos(pos, cond, pivot);
    }

    /**
     * Set window position to WindowPos.
     *
     * Parameter:
     *  pos  - predefined position of next window.
     *  cond - should be one of the following values:
     *         ImGuiCond_None = 0,
     *         ImGuiCond_Always = 1,
     *         ImGuiCond_Once = 2,
     *         ImGuiCond_FirstUseEver = 4,
     *         ImGuiCond_Appearing = 8
     *  gap  - gap between GLFW window and ImGui window.
     */
    void SetNextWindowPos(WindowPos pos, int cond = 0, float gap = 10.0f) {
        ContextControl control(imgui_context_);
        switch (pos) {
        case LEFT_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, gap), cond, ImVec2(0.0f, 0.0f));
            SetNextWindowSize(0.0f, window_->height() - gap - gap);
            break;
        case BOTTOM_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, window_->height() - gap), cond,
                                    ImVec2(0.0f, 1.0f));
            SetNextWindowSize(window_->width() - gap - gap, 0.0f);
            break;
        case TOP_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, gap), cond, ImVec2(0.0f, 0.0f));
            SetNextWindowSize(window_->width() - gap - gap, 0.0f);
            break;

        case LEFT_BOTTOM_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, window_->height() - gap),
                                    cond,
                                    ImVec2(0.0f, 1.0f));
            break;
        case LEFT_TOP_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, gap), cond, ImVec2(0.0f, 0.0f));
            break;
        case RIGHT_BOTTOM_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(window_->width() - gap,
                                           window_->height() - gap),
                                    cond,
                                    ImVec2(1.0f, 1.0f));
            break;
        case RIGHT_TOP_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(window_->width() - gap, gap), cond,
                                    ImVec2(1.0f, 0.0f));
            break;
        }
    }

    /**
     * Equal to ImGui::SetNextWindowSize().
     *
     * Parameter:
     *  size_x - size in x direction, set to 0.0f to force an auto-fit on this
     *           axis.
     *  size_x - size in x direction, set to 0.0f to force an auto-fit on this
     *           axis.
     *  cond   - should be one of the following values:
     *           ImGuiCond_None = 0,
     *           ImGuiCond_Always = 1,
     *           ImGuiCond_Once = 2,
     *           ImGuiCond_FirstUseEver = 4,
     *           ImGuiCond_Appearing = 8
     */
    void SetNextWindowSize(float size_x, float size_y, ImGuiCond cond = 0) {
        ContextControl control(imgui_context_);
        ImGui::SetNextWindowSize(ImVec2(size_x, size_y), cond);
    }

    /**
     * Equal to ImGui::SetNextWindowBgAlpha().
     */
    void SetNextWindowBgAlpha(float alpha) {
        ContextControl control(imgui_context_);
        ImGui::SetNextWindowBgAlpha(alpha);
    }

    /**
     * Return ImGui::IO of current context.
     */
    ImGuiIO& IO() {
        return imgui_context_->IO;
    }

    /**
     * Return ImGui::IO of current context.
     */
    const ImGuiIO& IO() const {
        return imgui_context_->IO;
    }

    /**
     * Equal to ImGui::PushStyleColor.
     */
    void PushStyleColor(ImGuiCol idx, const RGB32Color& color) {
        ContextControl control(imgui_context_);
        ImGui::PushStyleColor(idx, color.ToUInt());
    }

    /**
     * Equal to ImGui::PopStyleColor.
     */
    void PopStyleColor() {
        ContextControl control(imgui_context_);
        ImGui::PopStyleColor();
    }

    /**
     * Return the current window.
     */
    ImGuiWindow* CurrentWindow() const {
        return imgui_context_->CurrentWindow;
    }

    /**
     * Return the current window box.
     */
    RBox2D GetWindowBox() const {
        ImGuiWindow* window = imgui_context_->CurrentWindow;
        if (!window) return RBox2D();

        double y = window_->height() - window->Pos.y - window->Size.y;
        return RBox2D(window->Pos.x, window->Pos.x + window->Size.x,
                      y, y + window->Size.y);
    }

    /**
     * Get window box by name.
     */
    RBox2D GetWindowBox(const char* name) const {
        ContextControl control(imgui_context_);

        ImGuiID id = ImHashStr(name);
        ImGuiWindow* window = ImGui::FindWindowByID(id);
        if (!window) return RBox2D();

        double y = window_->height() - window->Pos.y - window->Size.y;
        return RBox2D(window->Pos.x, window->Pos.x + window->Size.x,
                      y, y + window->Size.y);
    }

    /**
     * Check if the mouse is hovered on any window.
     */
    bool IsMouseHoveredOnAnyWindow() {
        ContextControl control(imgui_context_);
        return ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
    }

    /**
     * This function is equivalent to calling ImGui::IsMousePosValid(nullptr) if
     * there is only one ImGui object.
     */
    bool IsMousePosValid() const {
        ContextControl control(imgui_context_);
        return ImGui::IsMousePosValid(nullptr);
    }

    /**
     * Check if the mouse is currently hovering over the current window.
     *
     * This function is equivalent to calling ImGui::IsWindowHovered() if there
     * is only one ImGui object.
     */
    bool IsWindowHovered() const {
        ContextControl control(imgui_context_);
        return ImGui::IsWindowHovered();
    }

    /**
     * Retrieve the render list associated with the current window for drawing
     * objects.
     *
     * Return a ui::RenderList object encapsulating the ImGui draw list for the
     * current window.
     */
    ui::RenderList GetWindowRenderList() const {
        ContextControl control(imgui_context_);
        return ui::RenderList(ImGui::GetWindowDrawList(), this->height());
    }

    /**
     * Retrieve the render list for drawing objects in the foreground.
     *
     * Return A ui::RenderList object encapsulating the ImGui draw list for the
     * foreground layer.
     */
    ui::RenderList GetForegroundRenderList() const {
        ContextControl control(imgui_context_);
        return ui::RenderList(ImGui::GetForegroundDrawList(), this->height());
    }

    /**
     * Get the pixel size of the given text.
     */
    void ComputeTextSize(const std::string& text, double* w, double* h) const {
        CHECK(w);
        CHECK(h);

        ContextControl control(imgui_context_);
        ImVec2 text_size = ImGui::CalcTextSize(text.c_str());
        *w = text_size.x;
        *h = text_size.y;
    }

    /**
     * Return the current mouse position.
     *
     * In order to be consistent with the geometric coordinates, the coordinates
     * we return are different from ImGui's mouse position. The Y coordinate is
     * reversed.
     */
    RPoint2D mouse_position() const {
        return RPoint2D(IO().MousePos.x, this->height() - IO().MousePos.y);
    }

    /**
     * Return true if the given mouse button is just pressed.
     */
    bool mouse_just_pressed(ImGuiMouseButton button) const {
        return IO().MouseDownDuration[button] == 0.0f;
    }

    /**
     * Return true if the given mouse button is pressed.
     */
    bool mouse_pressed(ImGuiMouseButton button) const {
        return IO().MouseDown[button];
    }

    /**
     * Return mouse moving vector between two frames.
     */
    RVector2D mouse_delta() const {
        return RVector2D(IO().MouseDelta.x, IO().MouseDelta.y);
    }

    /**
     * Return mouse wheel.
     */
    double mouse_wheel() const {
        return IO().MouseWheel;
    }

    /**
     * Return the current framerate.
     */
    float framerate() const {
        return IO().Framerate;
    }

    /**
     * Get font size.
     */
    float font_size() const {
        return font_size_;
    }

    /**
     * Return ImGui::Style() of current context.
     */
    ImGuiStyle& style() {
        return imgui_context_->Style;
    }

    /**
     * Return ImGui::Style() of current context.
     */
    const ImGuiStyle& style() const {
        return imgui_context_->Style;
    }

private:
    // Font size.
    float font_size_ = 0.0f;

    // Previous ImGui contex. Used for Begin() and End().
    ImGuiContext* previous_imgui_context_ = nullptr;
};

} // namespace cl

#endif // CODELIBRARY_UI_UI_H_
