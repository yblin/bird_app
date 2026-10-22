//
// Copyright 2022-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_GLFW_CALLBACK_H_
#define CODELIBRARY_UI_GLFW_CALLBACK_H_

#include <map>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "third_party/imgui/imgui.h"
#include "third_party/imgui/imgui_internal.h"

#if __has_include("GLFW/glfw3.h")
#include "GLFW/glfw3.h"
#else
#include "third_party/GLFW/glfw3.h"
#endif

#ifdef _WIN32
#undef APIENTRY
#ifndef GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#endif
#include <third_party/GLFW/glfw3native.h>   // for glfwGetWin32Window
#endif

#include "codelibrary/base/log.h"
#include "codelibrary/ui/key_controller.h"

namespace cl {
namespace ui {

/**
 * Bind ImGui to GLFW window.
 * One GLFW window can only match one UI.
 */
class WindowUIBinder {
public:
    static WindowUIBinder* GetInstance() {
        static WindowUIBinder binder;
        return &binder;
    }

    /**
     * Bind GLFW window and ImGui.
     */
    void Bind(GLFWwindow* window, ImGuiContext* context) {
        CHECK(window);
        CHECK(context);
        CHECK(ui_contexts_.find(window) == ui_contexts_.end());

        ui_contexts_[window] = context;
    }

    /**
     * Unbind ImGui to specific GLFW window.
     */
    void Unbind(GLFWwindow* window) {
        CHECK(window);
        CHECK(ui_contexts_.find(window) != ui_contexts_.end());

        ui_contexts_.erase(window);
    }

    /**
     * Get binded ImGui context.
     */
    ImGuiContext* GetUI(GLFWwindow* window) const {
        CHECK(window);

        auto itr = ui_contexts_.find(window);
        return itr == ui_contexts_.end() ? nullptr : itr->second;
    }

    /**
     * Get binded ImGui context, ensure it exists.
     */
    ImGuiContext* GetUIOrDie(GLFWwindow* window) const {
        CHECK(window);

        auto itr = ui_contexts_.find(window);
        CHECK(itr != ui_contexts_.end());

        return itr->second;
    }

private:
    WindowUIBinder() = default;

    std::map<GLFWwindow*, ImGuiContext*> ui_contexts_;
};

/**
 * Convert GLFW key to modifier.
 */
inline int KeyToModifier(int key) {
    if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL)
        return GLFW_MOD_CONTROL;
    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)
        return GLFW_MOD_SHIFT;
    if (key == GLFW_KEY_LEFT_ALT || key == GLFW_KEY_RIGHT_ALT)
        return GLFW_MOD_ALT;
    if (key == GLFW_KEY_LEFT_SUPER || key == GLFW_KEY_RIGHT_SUPER)
        return GLFW_MOD_SUPER;
    return 0;
}

/**
 * Add modefy keys to current ImGUI.
 */
inline void UpdateKeyModifiers(int mods, ImGuiContext* imgui_context) {
    CHECK(imgui_context);

    ImGuiIO& io = imgui_context->IO;
    io.AddKeyEvent(ImGuiKey_ModCtrl,  (mods & GLFW_MOD_CONTROL) != 0);
    io.AddKeyEvent(ImGuiKey_ModShift, (mods & GLFW_MOD_SHIFT)   != 0);
    io.AddKeyEvent(ImGuiKey_ModAlt,   (mods & GLFW_MOD_ALT)     != 0);
    io.AddKeyEvent(ImGuiKey_ModSuper, (mods & GLFW_MOD_SUPER)   != 0);
}


inline void WindowFocusCallback(GLFWwindow* window, int focused) {
    ImGuiContext* ui = WindowUIBinder::GetInstance()->GetUIOrDie(window);
    ui->IO.AddFocusEvent(focused != 0);
}

inline void MouseButtonCallback(GLFWwindow* window, int button, int action,
                                int mods) {
    ImGuiContext* ui = WindowUIBinder::GetInstance()->GetUIOrDie(window);
    UpdateKeyModifiers(mods, ui);

    if (button >= 0 && button < ImGuiMouseButton_COUNT) {
        ui->IO.AddMouseButtonEvent(button, action == GLFW_PRESS);
    }
}

inline void CursorPositionCallback(GLFWwindow* window, double x, double y) {
    ImGuiContext* ui = WindowUIBinder::GetInstance()->GetUIOrDie(window);
    ui->IO.AddMousePosEvent(static_cast<float>(x), static_cast<float>(y));
}

inline void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGuiContext* ui = WindowUIBinder::GetInstance()->GetUIOrDie(window);
    ui->IO.AddMouseWheelEvent(static_cast<float>(xoffset),
                              static_cast<float>(yoffset));
}

inline void KeyCallback(GLFWwindow* window, int keycode, int scancode,
                        int action, int mods) {
    if (action != GLFW_PRESS && action != GLFW_RELEASE) return;

    // Workaround: X11 does not include current pressed/released modifier
    // key in 'mods' flags.
    if (int keycode_to_mod = KeyToModifier(keycode))
        mods = (action == GLFW_PRESS) ? (mods | keycode_to_mod)
                                      : (mods & ~keycode_to_mod);

    ImGuiContext* ui = WindowUIBinder::GetInstance()->GetUIOrDie(window);
    UpdateKeyModifiers(mods, ui);

    ImGuiIO& io = ui->IO;
    ImGuiKey imgui_key = KeyController::GetInstance()->MapGLFWKey(keycode);
    io.AddKeyEvent(imgui_key, (action == GLFW_PRESS));
    // To support legacy indexing (<1.87 user code).
    io.SetKeyEventNativeData(imgui_key, keycode, scancode);
}

inline void CharCallback(GLFWwindow* window, unsigned int c) {
    WindowUIBinder::GetInstance()->GetUIOrDie(window)->IO.AddInputCharacter(c);
}

} // namespace ui
} // namespace cl

#endif // CODELIBRARY_UI_GLFW_CALLBACK_H_
