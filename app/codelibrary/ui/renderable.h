//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_RENDERABLE_H_
#define CODELIBRARY_UI_RENDERABLE_H_

#include <string>

#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/mesh/polygon_triangulation_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/image/image.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/shader.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/ui/glfw_callback.h"
#include "codelibrary/ui/window.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace ui {

/**
 * Implement UI rendering code here.
 */
class Renderable {
public:
    /**
     * Used to automatically restore the previous ImGuiContext after some
     * operations on a specific ImGuiContext.
     *
     * Usage:
     *   Func() {
     *       ContextControl control(context);
     *       Do something.....
     *   }
     */
    class ContextControl {
    public:
        explicit ContextControl(ImGuiContext* context) {
            CHECK(!current_context_);

            current_context_ = context;
            previous_context_ = ImGui::GetCurrentContext();
            ImGui::SetCurrentContext(current_context_);
        }

        ~ContextControl() {
            ImGui::SetCurrentContext(previous_context_);
        }

    private:
        ImGuiContext* current_context_ = nullptr;
        ImGuiContext* previous_context_ = nullptr;
    };

    Renderable() = delete;

    explicit Renderable(Window* window)
        : window_(window) {
        CHECK(window);

        imgui_context_ = ImGui::CreateContext();
        CHECK(imgui_context_);

        WindowUIBinder::GetInstance()->Bind(window->handle(), imgui_context_);
    }

    Renderable(const Renderable&) = delete;

    virtual ~Renderable() {
        WindowUIBinder::GetInstance()->Unbind(window_->handle());
        glDeleteVertexArrays(1, &vertex_array_);
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteBuffers(1, &element_buffer_);
        glDeleteTextures(1, &font_texture_);
        ImGui::DestroyContext(imgui_context_);
        imgui_context_ = nullptr;
    }

    Renderable& operator=(const Renderable&) = delete;

    /**
     * Start a new ImGui frame.
     *
     * It sets the current ImGui context to this one.
     */
    void NewFrame() {
        ContextControl control(imgui_context_);

        if (!shader_) Initialize();

        ImGuiIO& io = imgui_context_->IO;

        // Setup display size (every frame to accommodate for window resizing).
        int w = window_->width(), h = window_->height();
        int display_w = window_->framebuffer_width();
        int display_h = window_->framebuffer_height();
        io.DisplaySize = ImVec2(static_cast<float>(w), static_cast<float>(h));
        if (w > 0 && h > 0) {
            io.DisplayFramebufferScale =
                    ImVec2(static_cast<float>(display_w) / w,
                           static_cast<float>(display_h) / h);
        }

        // Setup time step.
        double current_time = glfwGetTime();
        io.DeltaTime = current_time_ > 0.0 ? static_cast<float>(current_time -
                                                                current_time_)
                                           : 1.0f / 60.0f;
        current_time_ = current_time;

        UpdateMouseData();
        UpdateMouseCursor();
        UpdateGamepads();

        // Call ImGui new frame.
        ImGui::NewFrame();
        ++n_frames_;
    }

    /**
     * Render function.
     */
    void Render() {
        ContextControl control(imgui_context_);

        // Set ImGui render data.
        ImGui::Render();

        this->RenderDrawData(ImGui::GetDrawData());
    }

    /**
     * Save ui framebuffer to image.
     */
    void SaveToImage(Image* image) const {
        CHECK(image);

        int w = width(), h = height();

        image->Reset(h, w, 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image->data());
        image->VerticalFlip();
    }

    /**
     * Return the height of parent window.
     */
    int height() const {
        return window_->height();
    }

    /**
     * Return the width of parent window.
     */
    int width() const {
        return window_->width();
    }

    /**
     * Return the current number of rendered frames.
     */
    int64_t n_frames() const {
        return n_frames_;
    }

private:
    /**
     * Render draw data.
     */
    void RenderDrawData(ImDrawData* draw_data) {
        // Avoid rendering when minimized, scale coordinates for retina displays
        // (screen coordinates != framebuffer coordinates).
        int fb_width = static_cast<int>(draw_data->DisplaySize.x *
                                        draw_data->FramebufferScale.x);
        int fb_height = static_cast<int>(draw_data->DisplaySize.y *
                                         draw_data->FramebufferScale.y);
        if (fb_width <= 0 || fb_height <= 0)
            return;

        // Backup GL state.
        GLint last_scissor_box[4];
        glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
        bool last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);

        this->SetupRenderState(draw_data, fb_width, fb_height);

        // Will project scissor/clipping rectangles into framebuffer space.
        // (0,0) unless using multi-viewports.
        ImVec2 clip_off = draw_data->DisplayPos;
        // (1,1) unless using retina display which are often (2,2).
        ImVec2 clip_scale = draw_data->FramebufferScale;

        // Render command lists
        for (int n = 0; n < draw_data->CmdListsCount; ++n) {
            const ImDrawList* cmd_list = draw_data->CmdLists[n];

            // Upload vertex/index buffers
            GLsizeiptr vertex_buffer_size =
                    static_cast<GLsizeiptr>(cmd_list->VtxBuffer.Size) *
                    sizeof(ImDrawVert);
            GLsizeiptr index_buffer_size =
                    static_cast<GLsizeiptr>(cmd_list->IdxBuffer.Size) *
                    sizeof(ImDrawIdx);

            if (vertex_buffer_size_ < vertex_buffer_size) {
                vertex_buffer_size_ = vertex_buffer_size;
                glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size_, nullptr,
                             GL_STREAM_DRAW);
            }
            if (index_buffer_size_ < index_buffer_size) {
                index_buffer_size_ = index_buffer_size;
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer_size_,
                             nullptr, GL_STREAM_DRAW);
            }
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_buffer_size,
                            (const GLvoid*)cmd_list->VtxBuffer.Data);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, index_buffer_size,
                            (const GLvoid*)cmd_list->IdxBuffer.Data);

            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; ++cmd_i) {
                const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
                if (pcmd->UserCallback) {
                    // User callback, registered via ImDrawList::AddCallback()
                    // (ImDrawCallback_ResetRenderState is a special callback
                    // value used by the user to request the renderer to reset
                    // render state.)
                    if (pcmd->UserCallback == ImDrawCallback_ResetRenderState)
                        SetupRenderState(draw_data, fb_width, fb_height);
                    else
                        pcmd->UserCallback(cmd_list, pcmd);
                } else {
                    // Project scissor/clipping rectangles into framebuffer
                    // space.
                    ImVec2 clip_min((pcmd->ClipRect.x - clip_off.x) *
                                    clip_scale.x,
                                    (pcmd->ClipRect.y - clip_off.y) *
                                    clip_scale.y);
                    ImVec2 clip_max((pcmd->ClipRect.z - clip_off.x) *
                                    clip_scale.x,
                                    (pcmd->ClipRect.w - clip_off.y) *
                                    clip_scale.y);
                    if (clip_max.x <= clip_min.x || clip_max.y <= clip_min.y)
                        continue;

                    // Apply scissor/clipping rectangle (Y is inverted in
                    // OpenGL)
                    glScissor(static_cast<int>(clip_min.x),
                              static_cast<int>(fb_height - clip_max.y),
                              static_cast<GLsizei>(clip_max.x - clip_min.x),
                              static_cast<GLsizei>(clip_max.y - clip_min.y));

                    // Bind texture, Draw.
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D,
                                  (GLuint)(intptr_t)pcmd->GetTexID());
                    glDrawElements(GL_TRIANGLES,
                                   (GLsizei)pcmd->ElemCount,
                                   sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT
                                                          : GL_UNSIGNED_INT,
                                   reinterpret_cast<void*>((intptr_t)(
                                       pcmd->IdxOffset * sizeof(ImDrawIdx))));
                }
            }
        }

        // Restore modified GL state.
        glScissor(last_scissor_box[0], last_scissor_box[1],
                  static_cast<GLsizei>(last_scissor_box[2]),
                  static_cast<GLsizei>(last_scissor_box[3]));
        if (last_enable_scissor_test)
            glEnable(GL_SCISSOR_TEST);
        else
            glDisable(GL_SCISSOR_TEST);
    }

    /**
     * Initialize, this function only needs to be called once.
     */
    void Initialize() {
        ImGuiIO& io = imgui_context_->IO;
        CHECK(!io.BackendRendererUserData)
                << "Already initialized a renderer backend!";

        InitializeGL();

        CHECK(!io.BackendPlatformUserData)
                << "Already initialized a platform backend!";

        // We can honor GetMouseCursor() values (optional).
        io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
        // We can honor io.WantSetMousePos requests (optional, rarely used).
        io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;
        io.SetClipboardTextFn = [](void* user_data, const char* text) {
            glfwSetClipboardString(reinterpret_cast<GLFWwindow*>(user_data),
                                   text);
        };
        io.GetClipboardTextFn = [](void* user_data) {
            return glfwGetClipboardString(
                      reinterpret_cast<GLFWwindow*>(user_data));
        };
        io.ClipboardUserData = window_->handle();

        // Set platform dependent data in viewport.
    #if defined(_WIN32)
        ImGui::GetMainViewport()->PlatformHandleRaw =
                reinterpret_cast<void*>(glfwGetWin32Window(window_->handle()));
    #endif

        // Create mouse cursors.
        // (By design, on X11 cursors are user configurable and some cursors may
        // be missing. When a cursor doesn't exist,
        // GLFW will emit an error which will often be printed by the app, so we
        // temporarily disable error reporting.
        // Missing cursors will return NULL and our UpdateMouseCursor()
        // function will use the Arrow cursor instead.)
        mouse_cursors_[ImGuiMouseCursor_Arrow] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_TextInput] =
                glfwCreateStandardCursor(GLFW_IBEAM_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNS] =
                glfwCreateStandardCursor(GLFW_VRESIZE_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeEW] =
                glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_Hand] =
                glfwCreateStandardCursor(GLFW_HAND_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeAll] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNESW] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNWSE] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_NotAllowed] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);

        // Install GLFW callback.
        glfwSetWindowFocusCallback(window_->handle(), WindowFocusCallback);
        glfwSetCursorPosCallback(window_->handle(), CursorPositionCallback);
        glfwSetMouseButtonCallback(window_->handle(), MouseButtonCallback);
        glfwSetScrollCallback(window_->handle(), ScrollCallback);
        glfwSetKeyCallback(window_->handle(), KeyCallback);
        glfwSetCharCallback(window_->handle(), CharCallback);
    }

    /**
     * Unlike SetupGL(), this function only needs to be called once.
     */
    void InitializeGL() {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec2 position;
            layout (location = 1) in vec2 uv;
            layout (location = 2) in vec4 color;

            uniform mat4 projection;
            out vec2 frag_uv;
            out vec4 frag_color;

            void main() {
                frag_uv = uv;
                frag_color = color;
                gl_Position = projection * vec4(position.xy, 0, 1);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            in vec2 frag_uv;
            in vec4 frag_color;
            uniform sampler2D tex;
            layout (location = 0) out vec4 out_color;
            void main() {
                out_color = frag_color * texture(tex, frag_uv);
            }
        );

        shader_ = gl::ShaderFactory::GetInstance()->Create(vertex_shader,
                                                           fragment_shader);

        texture_location_ = shader_->GetLocation("tex");
        projection_location_ = shader_->GetLocation("projection");

        // Create buffers.
        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &element_buffer_);

        CreateFontsTexture();
    }

    /**
     * Create fonts texture.
     */
    void CreateFontsTexture() {
        ImGuiIO& io = imgui_context_->IO;

        // Build texture atlas.
        unsigned char* pixels;
        int width, height;
        // Load as RGBA 32-bit (75% of the memory is wasted, but default font is
        // so small) because it is more likely to be compatible with user's
        // existing shaders. If your ImTextureId represent a higher-level
        // concept than just a GL texture id, consider calling
        // GetTexDataAsAlpha8() instead to save on GPU memory.
        io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

        // Upload texture to graphics system.
        // (Bilinear sampling is required by default.
        // Set 'io.Fonts->Flags |= ImFontAtlasFlags_NoBakedLines' or
        // 'style.AntiAliasedLinesUseTex = false' to allow point/nearest
        // sampling).
        GLint last_texture;
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
        glGenTextures(1, &font_texture_);
        glBindTexture(GL_TEXTURE_2D, font_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, pixels);

        // Store our identifier.
        io.Fonts->SetTexID((ImTextureID)(intptr_t)font_texture_);

        // Restore state.
        glBindTexture(GL_TEXTURE_2D, last_texture);
    }

    /**
     * Setup render state. It will be called every frame.
     */
    void SetupRenderState(ImDrawData* draw_data, int fb_width, int fb_height) {
        // Setup render state: alpha-blending enabled, no face culling,
        // no depth testing, scissor enabled, polygon fill.
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE,
                            GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_CULL_FACE);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        glEnable(GL_SCISSOR_TEST);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Setup viewport, orthographic projection matrix.
        // Our visible imgui space lies from draw_data->DisplayPos (top left) to
        // draw_data->DisplayPos+data_data->DisplaySize (bottom right).
        // DisplayPos is (0,0) for single viewport apps.
        glViewport(0, 0, fb_width, fb_height);
        float l = draw_data->DisplayPos.x;
        float r = draw_data->DisplayPos.x + draw_data->DisplaySize.x;
        float t = draw_data->DisplayPos.y;
        float b = draw_data->DisplayPos.y + draw_data->DisplaySize.y;
        gl::FTransform ortho_projection(
            2.0f / (r - l),    0.0f,               0.0f,   0.0f,
            0.0f,              2.0f / (t - b),     0.0f,   0.0f,
            0.0f,              0.0f,              -1.0f,   0.0f,
            (r + l) / (l - r), (t + b) / (b - t),  0.0f,   1.0f);

        shader_->Use();
        shader_->SetUniform(projection_location_, ortho_projection);
        shader_->SetUniform(texture_location_, 0);

        // Bind vertex/index buffers and setup attributes for ImDrawVert.
        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_);

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert),
                              reinterpret_cast<GLvoid*>(IM_OFFSETOF(ImDrawVert,
                                                                    pos)));
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert),
                              reinterpret_cast<GLvoid*>(IM_OFFSETOF(ImDrawVert,
                                                                    uv)));
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE,
                              sizeof(ImDrawVert),
                              reinterpret_cast<GLvoid*>(IM_OFFSETOF(ImDrawVert,
                                                                    col)));
    }

    /**
     * Update mouse data. It will be called every frame.
     */
    void UpdateMouseData() {
        ImGuiIO& io = imgui_context_->IO;

        if (window_->GetAttribute(GLFW_FOCUSED) != 0) {
            // (Optional) Set OS mouse position from Dear ImGui if requested
            // (rarely used, only when ImGuiConfigFlags_NavEnableSetMousePos is
            // enabled by user).
            if (io.WantSetMousePos) {
                window_->SetMousePosition(io.MousePos.x, io.MousePos.y);
            }
        }
    }

    /**
     * Update mouse cursor flags.
     */
    void UpdateMouseCursor() {
        ImGuiIO& io = imgui_context_->IO;
        if ((io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) ||
            window_->GetInputMode(GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
            return;

        ImGuiMouseCursor imgui_cursor = ImGui::GetMouseCursor();
        if (imgui_cursor == ImGuiMouseCursor_None || io.MouseDrawCursor) {
            // Hide OS mouse cursor if imgui is drawing it or if it wants no
            // cursor.
            window_->HideMouseCursor();
        } else {
            // Show OS mouse cursor.
            window_->SetMouseCursor(mouse_cursors_[imgui_cursor] ?
                                        mouse_cursors_[imgui_cursor] :
                                        mouse_cursors_[ImGuiMouseCursor_Arrow]);
            window_->ShowMouseCursor();
        }
    }

    /**
     * Update gamepad inputs.
     */
    void UpdateGamepads() {
        ImGuiIO& io = ImGui::GetIO();
        if ((io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad) == 0)
            return;

        io.BackendFlags &= ~ImGuiBackendFlags_HasGamepad;
        int axes_count = 0, buttons_count = 0;
        const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axes_count);
        const unsigned char* buttons =
                window_->GetJoystickButtons(&buttons_count);
        if (axes_count == 0 || buttons_count == 0)
            return;

        io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
        MapGamepadButton(ImGuiKey_GamepadStart, buttons_count, 6, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceDown, buttons_count, 0, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceRight, buttons_count, 1, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceLeft, buttons_count, 2, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceUp, buttons_count, 3, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadLeft, buttons_count, 13, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadRight, buttons_count, 11, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadUp, buttons_count, 10, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadDown, buttons_count, 12, buttons);
        MapGamepadButton(ImGuiKey_GamepadL1, buttons_count, 4, buttons);
        MapGamepadButton(ImGuiKey_GamepadR1, buttons_count, 5, buttons);
        MapGamepadAnalog(ImGuiKey_GamepadL2, axes_count, 4, axes, -0.75f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadR2, axes_count, 5, axes, -0.75f, 1.0f);
        MapGamepadButton(ImGuiKey_GamepadL3, buttons_count, 8, buttons);
        MapGamepadButton(ImGuiKey_GamepadR3, buttons_count, 9, buttons);
        MapGamepadAnalog(ImGuiKey_GamepadLStickLeft, axes_count, 0, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickRight, axes_count, 0, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickUp, axes_count, 1, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickDown, axes_count, 1, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickLeft, axes_count, 2, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickRight, axes_count, 2, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickUp, axes_count, 3, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickDown, axes_count, 3, axes,
                         0.25f, 1.0f);
    }

    /**
     * Used for UpdateGamepads().
     */
    void MapGamepadButton(ImGuiKey key, int buttons_count, int button_no,
                          const unsigned char* buttons) {
        ImGuiIO& io = ImGui::GetIO();
        io.AddKeyEvent(key, (buttons_count > button_no &&
                             buttons[button_no] == GLFW_PRESS));
    }

    /**
     * Used for UpdateGamepads().
     */
    void MapGamepadAnalog(ImGuiKey key, int axes_count, int axis_no,
                          const float* axes, float v0, float v1) {
        ImGuiIO& io = ImGui::GetIO();
        float v = (axes_count > axis_no) ? axes[axis_no] : v0;
        v = (v - v0) / (v1 - v0);
        io.AddKeyAnalogEvent(key, v > 0.10f, Clamp(v, 0.0f, 1.0f));
    }

    // Font texture.
    GLuint font_texture_ = 0;

    // Shader to draw UI.
    gl::Shader* shader_ = nullptr;

    // Location of uniform paramters.
    GLint texture_location_ = 0;
    GLint projection_location_ = 0;

    // Rendering data for OpenGL.
    GLuint vertex_array_ = 0;
    GLuint vertex_buffer_ = 0;
    GLuint element_buffer_ = 0;

    // Buffer size.
    GLsizeiptr vertex_buffer_size_ = 0;
    GLsizeiptr index_buffer_size_ = 0;

    // Mouse cursors.
    GLFWcursor* mouse_cursors_[ImGuiMouseCursor_COUNT];

    // Current time.
    double current_time_ = 0.0;

    // Number of rendered frames.
    int64_t n_frames_ = 0;

protected:
    // The parent window.
    Window* window_ = nullptr;

    // Context for ImGui.
    ImGuiContext* imgui_context_ = nullptr;
};

} // namespace ui
} // namespace cl

#endif // CODELIBRARY_UI_RENDERABLE_H_
