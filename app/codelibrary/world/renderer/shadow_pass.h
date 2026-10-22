//
// Copyright 2023-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_

#include <algorithm>
#include <string>

#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/geometry/transform_3d.h"
#include "codelibrary/opengl/depth_texture.h"
#include "codelibrary/world/scene.h"

namespace cl {
namespace world {

/**
 * Cascade shadow map.
 */
struct CascadeShadowMap {
    /**
     * Splitted light frustum.
     */
    struct LightFrustum {
        // Light space matrix.
        gl::RTransform projection_view;

        // Z range in camera space.
        double camera_z_near, camera_z_far;

        // Depth texture.
        GLuint depth_texture = 0;

        // Scene nodes that intersected with this frustum.
        Array<Node*> nodes;
    };

    // Number of cascade levels.
    int n_cascade_levels = 0;

    // Resolution of the shadow map.
    int resolution = 0;

    // Shadow light direction.
    RVector3D direction;

    // Depth textures in light view space.
    Array<gl::DepthTexture> depth_textures;

    // Splitted light frustums.
    Array<LightFrustum> light_frustums;
};

/**
 * Shadow pass.
 */
class ShadowPass {
    // Default shadow map resolution.
    // Only for directional light currently.
    static constexpr int SHADOW_MAP_RESOLUTION = 2048;

    // Cascade number.
    static constexpr int MAX_N_CASCADE_LEVELS = 8;

    ShadowPass() {
        InitializeShader();
        ResetCascadeLevels(3);
        ResetResolution(SHADOW_MAP_RESOLUTION);
    }

    using Camera = gl::Camera<double>;

public:
    static ShadowPass* GetInstance() {
        static ShadowPass instance;
        return &instance;
    }

    /**
     * Input scene, output shadow map.
     *
     * Return false if no shadow map is created.
     */
    bool Pass(const Camera& camera, Scene* scene) {
        CHECK(scene);

        auto light = scene->lights().directional_light;
        if (!light) return false;
        if (!scene->show_shadow()) return false;

        Array<Node*> shadow_nodes;
        for (Node* n : scene->nodes()) {
            if (n->render_mode() == RenderMode::NO_RENDER) continue;

            if (n->is_visible() && (n->cast_shadow() || n->receive_shadow())) {
                shadow_nodes.push_back(n);
            }
        }

        CreateShadowMap(camera, shadow_nodes, light->direction());
        return true;
    }

    /**
     * Reset the resolution of the shadow map.
     */
    void ResetResolution(int resolution) {
        CHECK(resolution > 0);
        CHECK(resolution <= 32768);

        if (shadow_map_.resolution == resolution) return;

        shadow_map_.resolution = resolution;
        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            shadow_map_.depth_textures[i].Reset(resolution, resolution);
        }
        framebuffer_.Reset(resolution, resolution);
    }

    /**
     * Reset the number of cascade levels.
     */
    void ResetCascadeLevels(int n_cascade_levels) {
        CHECK(n_cascade_levels > 0);
        CHECK(n_cascade_levels <= MAX_N_CASCADE_LEVELS);

        if (shadow_map_.n_cascade_levels == n_cascade_levels) return;

        shadow_map_.n_cascade_levels = n_cascade_levels;
        shadow_map_.depth_textures.resize(n_cascade_levels);
        for (int i = 0; i < n_cascade_levels; ++i) {
            shadow_map_.depth_textures[i].Reset(shadow_map_.resolution,
                                                shadow_map_.resolution);
        }
    }

    /**
     * Set bias to fine-tune the position and definition of the shadow.
     */
    void set_bias(double bias) {
        CHECK(bias >= 0.0);

        bias_ = bias;
    }

    /**
     * Get a reasonable CSM split values by adjusting lambda value.
     */
    void set_split_lambda(double lambda) {
        CHECK(lambda > 0.1 && lambda <= 1.0);

        split_lambda_ = lambda;
    }

    /**
     * Return the computed shadow map.
     */
    const CascadeShadowMap& shadow_map() const {
        return shadow_map_;
    }

    /**
     * return the bias value of the shadow.
     */
    double bias() const {
        return bias_;
    }

private:
    /**
     * Initialize shader for computing shadow map.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 4) in mat4 instance_matrix;

            uniform bool is_instance;
            uniform mat4 projection_view;
            uniform mat4 model;

            void main() {
                if (is_instance) {
                    gl_Position = projection_view * model * instance_matrix *
                                  vec4(pos, 1);
                } else {
                    gl_Position = projection_view * model * vec4(pos, 1);
                }
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            void main() {}
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Create the directional shadow map.
     */
    void CreateShadowMap(const Camera& camera, const Array<Node*>& nodes,
                         const RVector3D& direction) {
        if (camera.type() == gl::CameraProjectionType::UNKNOWN) {
            LOG(INFO) << "Camera has not been set up.";
            return;
        }

        shadow_map_.direction = direction;
        ComputeLightFrutums(camera, nodes, direction);

        // Store viewport.
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        glViewport(0, 0, shadow_map_.resolution, shadow_map_.resolution);
        glEnable(GL_DEPTH_CLAMP);

        framebuffer_.Bind();

        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            framebuffer_.AttachDepthTexture(shadow_map_.depth_textures[i].id());
            const auto& frustum = shadow_map_.light_frustums[i];

            // Render to the depth map.
            glClear(GL_DEPTH_BUFFER_BIT);
            glCullFace(GL_FRONT);

            shader_->Use();
            shader_->SetUniform("projection_view", frustum.projection_view);
            for (Node* node : frustum.nodes) {
                shader_->SetUniform("model", node->global_transform());
                shader_->SetUniform("is_instance", node->is_instance());
                node->Render();
            }
            glCullFace(GL_BACK);
        }
        framebuffer_.Unbind();

        // Reset viewport.
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glDisable(GL_DEPTH_CLAMP);

        return;
    }

    /**
     * Split light's frustum according to the distance from scene to the
     * camera.
     */
    void ComputeLightFrutums(const Camera& camera, const Array<Node*>& nodes,
                             RVector3D direction) {
        Array<double> split_z(shadow_map_.n_cascade_levels + 1);
        split_z[0] = camera.z_near();
        for (int i = 1; i <= shadow_map_.n_cascade_levels; ++i) {
            double p = static_cast<double>(i) / shadow_map_.n_cascade_levels;
            double z1 = camera.z_near() * std::pow(camera.z_far() /
                                                   camera.z_near(),
                                                   p);
            double z2 = camera.z_near() + (camera.z_far() -
                                           camera.z_near()) * p;
            double z = split_lambda_ * (z1 - z2) + z2;
            split_z[i] = Clamp(z, camera.z_near(), camera.z_far());
        }

        // Compute light view.
        RPoint3D origin(0.0, 0.0, 0.0);
        RPlane3D plane(origin, direction);
        RVector3D z_axis(0.0, 0.0, 1.0);
        RVector3D right = direction.cross(z_axis).normalized();
        if (right.norm() < DBL_EPSILON) {
            right = RVector3D(1.0, 0.0, 0.0);
        }
        RVector3D up = right.cross(direction);

        // Important! The nodes in light frustums must be cleared.
        shadow_map_.light_frustums.clear();
        shadow_map_.light_frustums.resize(shadow_map_.n_cascade_levels);
        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            auto& light_frustum = shadow_map_.light_frustums[i];
            light_frustum.camera_z_near = split_z[i];
            light_frustum.camera_z_far = split_z[i + 1];
            light_frustum.depth_texture = shadow_map_.depth_textures[i].id();

            Array<RPoint3D> camera_frustum;
            camera.GetFrustum(split_z[i], split_z[i + 1], &camera_frustum);
            if (camera_frustum.empty()) continue;

            // Convert camera frustum to viewing box in light space.
            Array<RPoint3D> projection;
            for (const RPoint3D& p : camera_frustum) {
                RPoint3D q = geometry::Project(p, plane);
                double dis = SignedDistance(p, plane);
                projection.emplace_back(q.ToVector().dot(right),
                                        q.ToVector().dot(up),
                                        static_cast<float>(dis));
            }
            RBox3D camera_box(projection.begin(), projection.end());
            if (camera_box.empty()) continue;

            // Expanded camera viewing box.
            // Nodes that are not in the camera frustum may also cast shadows.
            RBox3D expanded_camera_box(camera_box.x_min(), camera_box.x_max(),
                                       camera_box.y_min(), camera_box.y_max(),
                                       -DBL_MAX,           camera_box.z_max());

            RBox3D cast_box, receive_box;
            for (Node* node : nodes) {
                RBox3D box = node->GetBoundingBox();
                if (box.empty()) continue;

                Array<RPoint3D> ps;
                for (int i = 0; i < 2; ++i) {
                    double x = i == 0 ? box.x_min() : box.x_max();
                    for (int j = 0; j < 2; ++j) {
                        double y = j == 0 ? box.y_min() : box.y_max();
                        for (int k = 0; k < 2; ++k) {
                            double z = k == 0 ? box.z_min() : box.z_max();
                            RPoint3D p(x, y, z);
                            p = node->global_transform()(p);
                            RPoint3D q = geometry::Project(p, plane);
                            double dis = SignedDistance(p, plane);
                            ps.emplace_back(q.ToVector().dot(right),
                                            q.ToVector().dot(up),
                                            dis);
                        }
                    }
                }

                // Compute node's bounding box in light space.
                RBox3D box1 = RBox3D(ps.begin(), ps.end());
                if (node->cast_shadow() &&
                    geometry::Intersect(box1, expanded_camera_box)) {
                    light_frustum.nodes.push_back(node);
                    cast_box.Union(box1);
                }

                if (node->receive_shadow() &&
                    geometry::Intersect(box1, camera_box)) {
                    receive_box.Union(box1);
                }
            }
            if (cast_box.empty() || receive_box.empty()) {
                light_frustum.nodes.clear();
                continue;
            }

            RBox2D camera_box_2d(camera_box.x_min(), camera_box.x_max(),
                                 camera_box.y_min(), camera_box.y_max());
            RBox2D cast_box_2d(cast_box.x_min(), cast_box.x_max(),
                               cast_box.y_min(), cast_box.y_max());
            RBox2D receive_box_2d(receive_box.x_min(), receive_box.x_max(),
                                  receive_box.y_min(), receive_box.y_max());
            RBox2D viewing;
            if (!geometry::Intersect(cast_box_2d, receive_box_2d, &viewing) ||
                !geometry::Intersect(camera_box_2d, viewing, &viewing)) {
                light_frustum.nodes.clear();
                continue;
            }

            // Note that light frustum should cover:
            // 1. the camera's frustum;
            // 2. the cast shadow nodes that in front of the camera's frustum in
            //    light view space.
            // 3. the receive shadow nodes that in the camera's frustum.
            double z_near = cast_box.z_min();
            double z_far  = std::min(camera_box.z_max(), receive_box.z_max());

            if (z_near >= z_far) continue;

            RPoint3D pos = origin + direction * (z_near - 0.1);
            z_far -= (z_near - 0.1f);
            z_near = 0.1f;

            gl::Camera light_view(pos, pos + direction, up);
            light_view.SetOrthoModel(viewing.x_min(),
                                     viewing.x_max(),
                                     viewing.y_min(),
                                     viewing.y_max(),
                                     z_near,
                                     z_far);
            light_frustum.projection_view = light_view.projection() *
                                            light_view.viewing();
        }
    }

    // Used to alter the split value.
    double split_lambda_ = 0.9;

    // Bias for shadow.
    double bias_ = 1.0;

    // Shader for computing the shadow map.
    gl::Shader* shader_ = nullptr;

    // Framebuffer to draw the depth map.
    gl::Framebuffer framebuffer_;

    // Shadow map.
    CascadeShadowMap shadow_map_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_
