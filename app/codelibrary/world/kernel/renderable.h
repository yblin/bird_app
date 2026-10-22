//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_
#define CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_

#include <string>

#include "codelibrary/opengl/camera.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/material/material.h"

namespace cl {
namespace world {

enum class RenderMode {
    NORMAL = 0,         // Normal rendering mode.
    USE_OWN_SHADER = 1, // Render this node using own shader.
    NO_RENDER = 2       // Do not render this node.
                        // Note that it is not equal to is_visible_ = false.
};

/**
 * Class to implement rendering.
 *
 * It defines the basic rendering framework for 'world::Node'.
 */
struct Renderable {
    using Transform = gl::RTransform;
    using RenderList = Array<RenderObject*>;
    using Camera = gl::Camera<double>;

public:
    Renderable() = default;

    /**
     * Construction with a specific render object.
     */
    explicit Renderable(RenderObject* object) {
        CHECK(object);

        render_list_.push_back(object);
    }

    Renderable(const Renderable&) = delete;

    virtual ~Renderable() = default;

    Renderable& operator=(const Renderable&) = delete;

    /**
     * Call it before rendering.
     */
    virtual void BeforeRender() {}

    /**
     * Call it after rendering.
     */
    virtual void AfterRender() {}

    /**
     * Render this node.
     */
    virtual void Render() {
        if (!is_visible_) return;

        BeforeRender();
        for (RenderObject* o : render_list_) {
            o->Render();
        }
        AfterRender();
    }

    /**
     * Use own shader for rendering.
     */
    virtual void RenderWithOwnShader(const Camera& camera) {
        if (!is_visible_) return;
        if (render_list_.empty()) return;

        if (!shader_) InitializeShader();
        CHECK(shader_);
        SetShaderParameters(camera);

        BeforeRender();
        for (RenderObject* o : render_list_) {
            o->Render();
        }
        AfterRender();
    }

    /**
     * Clear the render list.
     */
    void ClearRenderList() {
        render_list_.clear();
    }

    /**
     * Add a render object to render list.
     */
    void AddRenderObject(RenderObject* render_object) {
        render_list_.push_back(render_object);
    }

    /**
     * Remove a render object from render list.
     */
    void RemoveRenderObject(RenderObject* render_object) {
        render_list_.erase(render_object);
    }

    /**
     * Set vertices color for all render objects.
     */
    void SetColor(const RGB32Color& color) {
        for (auto o : render_list_) {
            o->SetColor(color);
        }
    }

    virtual int n_vertices() const {
        int n_vertices = 0;
        for (RenderObject* o : render_list_) {
            n_vertices += o->n_vertices();
        }
        return n_vertices;
    }

    virtual int n_faces() const {
        int n_faces = 0;
        for (RenderObject* o : render_list_) {
            n_faces += o->n_faces();
        }
        return n_faces;
    }

    // O(1) functions to access the members.
    void show(bool is_visible = true)         { is_visible_ = is_visible;  }
    void hide()                               { is_visible_ = false;       }
    void set_visible(bool flag)               { is_visible_ = flag;        }
    bool is_visible()                   const { return is_visible_;        }
    RenderMode render_mode()            const { return render_mode_;       }
    void set_render_mode(RenderMode mode)     { render_mode_ = mode;       }
    const Transform& global_transform() const { return global_transform_;  }
    const RenderList& render_list()     const { return render_list_;       }
    Material* material()                      { return material_;          }
    const Material* material()          const { return material_;          }
    void set_material(Material* material)     { material_ = material;      }
    int render_order()                  const { return render_order_;      }
    void set_render_order(int order)          { render_order_ = order;     }

protected:
    virtual void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 1) in vec4 color;
            layout (location = 2) in vec3 normal;

            uniform mat4 view;
            uniform mat4 projection;
            uniform mat4 model;

            out vec4 frag_color;

            void main() {
                frag_color = color;
                gl_Position = projection * view * model * vec4(pos, 1.0);
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            in vec4 frag_color;

            out vec4 out_color;

            // Material parameters.
            uniform bool use_vertex_color;
            uniform vec4 albedo;

            void main() {
                out_color = use_vertex_color ? frag_color : albedo;
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    virtual void SetShaderParameters(const Camera& camera) {
        shader_->Use();
        shader_->SetUniform("view",       camera.viewing());
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("model",      global_transform_);

        if (material_) {
            shader_->SetUniform("use_vertex_color",
                                material_->use_vertex_color);
            shader_->SetUniform("albedo", material_->albedo);
        } else {
            shader_->SetUniform("use_vertex_color", true);
        }
    }

    // Render mode.
    RenderMode render_mode_ = RenderMode::NORMAL;

    // Node is visible or not.
    bool is_visible_ = true;

    // Render order of this node in the scene.
    int render_order_ = 0;

    // A render list consists of render objects. Note that it does not own
    // these objects.
    RenderList render_list_;

    // Own shader.
    gl::Shader* shader_ = nullptr;

    // Material of this node.
    Material* material_ = nullptr;

    // Transform from the local coordinate to global coordinate.
    // It will be automatically set by Scene class.
    Transform global_transform_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_
