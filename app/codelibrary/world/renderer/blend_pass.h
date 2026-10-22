//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_BLEND_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_BLEND_PASS_H_

#include <string>

#include "codelibrary/opengl/camera.h"
#include "codelibrary/opengl/depth_texture.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/opengl/texture.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * Blend to textures into one.
 */
class BlendPass {
    BlendPass() {
        InitializeShader();
        screen_quad_.SetRenderData(Quad());
    }

    using Camera = gl::Camera<double>;

public:
    static BlendPass* GetInstance() {
        static BlendPass instance;
        return &instance;
    }

    /**
     * Input background texture and light texture, output blended texture with
     * background and light.
     */
    void Pass(const gl::Texture& texture1,
              const gl::DepthTexture& depth_texture1,
              const gl::Texture& texture2,
              const gl::DepthTexture& depth_texture2) {
        shader_->Use();
        texture1.Bind(0);
        depth_texture1.Bind(1);
        texture2.Bind(2);
        depth_texture2.Bind(3);
        screen_quad_.Render();
    }

private:
    /**
     * Initialize outline shader.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 3) in vec2 texcoord;

            out vec2 frag_uv;

            void main() {
                frag_uv = texcoord;
                gl_Position = vec4(pos, 1);
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            layout(binding = 0) uniform sampler2D texture1;
            layout(binding = 1) uniform sampler2D depth_texture1;
            layout(binding = 2) uniform sampler2D texture2;
            layout(binding = 3) uniform sampler2D depth_texture2;

            in vec2 frag_uv;
            out vec4 frag_color;

            void main() {
                vec4 color1 = texture(texture1, frag_uv);
                vec4 color2 = texture(texture2, frag_uv);
                float d1 = texture(depth_texture1, frag_uv).r;
                float d2 = texture(depth_texture2, frag_uv).r;
                frag_color = (d1 < d2) ? color1 : color2;
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    // Shader from textures to outline.
    gl::Shader* shader_ = nullptr;

    // Help to render to the framebuffer.
    RenderObject screen_quad_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_BLEND_PASS_H_
