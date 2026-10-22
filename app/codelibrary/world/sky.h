//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_SKY_H_
#define CODELIBRARY_WORLD_SKY_H_

#include <algorithm>
#include <cmath>
#include <string>

#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/world/cubemap.h"
#include "codelibrary/world/light/light.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/primitive/cube_sphere.h"
#include "codelibrary/world/sky_data_rgb.inl"

namespace cl {
namespace world {

/**
 * Sky dome for 3D world.
 *
 * Reference:
 *   Hosek L, Wilkie A. An analytic model for full spectral sky-dome
 *   radiance[J]. ACM Transactions on Graphics, 2012, 31(4):1-9.
 */
class Sky : public Node {
    static constexpr int SUN_RADIANCE_D = 16;

public:
    /**
     * Initialize a cubemap sky.
     *
     * Parameters:
     *   sun_position       - the relative position of sun.
     *   cubemap_resolution - the resolution of cubemap, note that the sky will
     *                        be render into cubemap.
     */
    Sky(const RVector3D& sun_position = RVector3D(1.0, 1.0, 1.0),
        int cubemap_resolution = 1024)
        : Node("Sky"),
          cubemap_resolution_(cubemap_resolution),
          sun_light_("Sun light"),
          ambient_light_("Sky ambient light"),
          cubemap_("Sky cubemap") {
        CHECK(cubemap_resolution_ > 0);

        Initialize();

        sun_light_.set_type(Light::DIRECTIONAL);
        ambient_light_.set_type(Light::AMBIENT);
        this->AddNode(&sun_light_);
        this->AddNode(&ambient_light_);
        this->AddNode(&cubemap_);

        sun_position_ = sun_position.normalized();
        sky_sphere_.SetRenderData(CubeSphere(4));

        this->Update();
    }

    /**
     * Initialize a sphere sky, it can be move with scene.
     *
     * Parameters:
     *   radius       - the radius of the sky.
     *   sun_position - the relative position of sun.
     */
    Sky(double radius,
        const RVector3D& sun_position = RVector3D(1.0, 1.0, 1.0))
        : Node("Sky"),
          sun_light_("Sun light"),
          ambient_light_("Sky ambient light") {
        Initialize();

        sun_light_.set_type(Light::DIRECTIONAL);
        ambient_light_.set_type(Light::AMBIENT);
        this->AddNode(&sun_light_);
        this->AddNode(&ambient_light_);

        sun_position_ = sun_position.normalized();
        sky_sphere_.SetRenderData(CubeSphere(4));
        this->SetScale(radius);

        this->Update();
        this->AddRenderObject(&sky_sphere_);
    }

    /**
     * Set relative position of sun.
     */
    void SetSunPosition(const RVector3D& sun_position) {
        sun_position_ = sun_position.normalized();
        this->Update();
    }

    /**
     * Set sun position according to its polar angle and azimuth angle.
     */
    void SetSunPosition(double polar_angle, double azimuth_angle) {
        double theta = DegreeToRadian(polar_angle);
        double phi   = DegreeToRadian(azimuth_angle);
        sun_position_ = RVector3D(std::sin(theta) * std::cos(phi),
                                  std::sin(theta) * std::sin(phi),
                                  std::cos(theta));
        this->Update();
    }

    /**
     * Set the albedo of atmosphere. The range is [0, 1].
     */
    void SetAlbedo(double albedo) {
        CHECK(albedo >= 0.0 && albedo <= 1.0);

        albedo_ = albedo;
        this->Update();
    }

    /**
     * Set the turbidity of atmosphere. The range should be [2, 12].
     */
    void SetTurbidity(double turbidity) {
        CHECK(turbidity >= 2.0 && turbidity <= 12.0);

        turbidity_ = turbidity;
        this->Update();
    }

    /**
     * Get color of the sky at the given direction.
     */
    RGB32Color GetColor(const RVector3D& v) const {
        RVector3D s = GetRadiance(v);
        s *= rgb_normalize_;

        return RGB32Color(s.x, s.y, s.z);
    }

    /**
     * Get radiance of the sky at the given direction.
     */
    RVector3D GetRadiance(const RVector3D& v) const {
        RVector3D v1 = v.normalized();

        double cos_theta = Clamp(v1.z, 0.0, 1.0);
        double cos_gamma = Clamp(v1.dot(sun_position_), 0.0, 1.0);
        double gamma = std::acos(cos_gamma);
        RVector3D s = HosekWilkie(cos_theta, gamma, cos_gamma);
        return s * coefficents_[9];
    }

    /**
     * Return (unnormalized) RGB radiance of sun.
     */
    RVector3D GetSunRadiance() const {
        double cos_theta = std::cos(sun_theta_);

        if (cos_theta < 0.0)
            return RVector3D(0.0, 0.0, 0.0);

        double s = Clamp(cos_theta, 0.0, 0.999999);
        double t = Clamp((turbidity_ - 2.0) / 10.0, 0.0, 0.999999);
        s *= SUN_RADIANCE_D - 1;
        t *= SUN_RADIANCE_D - 1;

        int si0 = static_cast<int>(std::floor(s));
        int ti0 = static_cast<int>(std::floor(t));
        int si1 = (si0 + 1);
        int ti1 = (ti0 + 1);

        double sf = s - si0;
        double tf = t - ti0;

        ti0 *= SUN_RADIANCE_D;
        ti1 *= SUN_RADIANCE_D;

        RVector3D c = sky::sun_radiance[si0 + ti0] * (1.0 - sf) * (1.0 - tf) +
                      sky::sun_radiance[si1 + ti0] *        sf  * (1.0 - tf) +
                      sky::sun_radiance[si0 + ti1] * (1.0 - sf) *        tf  +
                      sky::sun_radiance[si1 + ti1] *        sf  *        tf;
        c *= 6.79998e-05;

        return c;
    }

    /**
     * Return the ambient color (use for ambient light).
     */
    RVector3D GetAmbientColor() const {
        RVector3D z(0.0, 0.0, 1.0);
        RVector3D v = z.cross(sun_position_);
        v = z.cross(v);

        return GetRadiance(v) * rgb_normalize_;
    }

    /**
     * Convert the sky model to (SDR) cubemap.
     */
    void ToCubemap(int resolution, Cubemap* cubemap) {
        ToCubemap(resolution, false, cubemap);
    }

    /**
     * Convert the sky model to (HDR) cubemap.
     */
    void ToHDRCubemap(int resolution, Cubemap* cubemap) {
        ToCubemap(resolution, true, cubemap);
    }

    Light* sun_light()                    { return &sun_light_;   }
    const RVector3D& sun_position() const { return sun_position_; }
    double albedo()                 const { return albedo_;       }
    double turbidity()              const { return turbidity_;    }

private:
    void SetShaderParameters(const Camera& camera) override {
        shader_->Use();
        shader_->SetUniform("view", camera.viewing());
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("model", global_transform_);
        SetParameters(false);
    }

    /**
     * Initialize node parameters.
     */
    void Initialize() {
        render_mode_ = RenderMode::USE_OWN_SHADER;
        cast_shadow_ = false;
        receive_shadow_ = false;
    }

    /**
     * Set shader parameters.
     */
    void SetParameters(bool to_hdr = false) const {
        shader_->Use();
        shader_->SetUniform("A", coefficents_[0]);
        shader_->SetUniform("B", coefficents_[1]);
        shader_->SetUniform("C", coefficents_[2]);
        shader_->SetUniform("D", coefficents_[3]);
        shader_->SetUniform("E", coefficents_[4]);
        shader_->SetUniform("F", coefficents_[5]);
        shader_->SetUniform("G", coefficents_[6]);
        shader_->SetUniform("H", coefficents_[7]);
        shader_->SetUniform("I", coefficents_[8]);
        shader_->SetUniform("Z", coefficents_[9]);
        shader_->SetUniform("N", rgb_normalize_);
        shader_->SetUniform("to_hdr", to_hdr);
        shader_->SetUniform("sun_direction", sun_position_);
    }

    /**
     * It should be called when the parameters are changed.
     */
    void Update() {
        sun_theta_ = std::acos(Clamp(sun_position_.z, 0.0, 1.0));

        for (int i = 0; i < 3; ++i) {
            coefficents_[0][i] = Evaluate(sky::datasets_rgb[i] + 0, 9);
            coefficents_[1][i] = Evaluate(sky::datasets_rgb[i] + 1, 9);
            coefficents_[2][i] = Evaluate(sky::datasets_rgb[i] + 2, 9);
            coefficents_[3][i] = Evaluate(sky::datasets_rgb[i] + 3, 9);
            coefficents_[4][i] = Evaluate(sky::datasets_rgb[i] + 4, 9);
            coefficents_[5][i] = Evaluate(sky::datasets_rgb[i] + 5, 9);
            coefficents_[6][i] = Evaluate(sky::datasets_rgb[i] + 6, 9);

            // Swapped in the dataset.
            coefficents_[7][i] = Evaluate(sky::datasets_rgb[i] + 8, 9);
            coefficents_[8][i] = Evaluate(sky::datasets_rgb[i] + 7, 9);

            // Radiances.
            coefficents_[9][i] = Evaluate(sky::datasets_rgb_rad[i], 1);
        }

        // Compute the radiances of sun.
        RVector3D s = HosekWilkie(std::cos(sun_theta_), 0.0, 1.0);
        s *= coefficents_[9];

        RVector3D convert = RVector3D(0.2126, 0.7152, 0.0722);
        const double illumination = s.dot(convert);

        // We can change the sum amount here.
        double sun_amount = std::fmod(2.0 * std::asin(sun_position_.z) /
                                      numbers::pi,
                                      4.0);
        if (sun_amount > 2.0)
            sun_amount = 0.0;
        if (sun_amount > 1.0)
            sun_amount = 2.0 - sun_amount;
        else if (sun_amount < -1.0)
            sun_amount = -2.0 - sun_amount;
        sun_amount = 0.6 + 0.45 * sun_amount;
        rgb_normalize_ = sun_amount / illumination;

        if (cubemap_resolution_ > 0)
            ToCubemap(cubemap_resolution_, false, &cubemap_);
        UpdateLight();
    }

    /**
     * Setup the shader.
     */
    void InitializeShader() override {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;

            uniform mat4 view;
            uniform mat4 projection;
            uniform mat4 model;

            out vec3 sky_texcoord;

            void main() {
                sky_texcoord = pos;
                gl_Position = projection * view * model * vec4(pos, 1);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            in vec3 sky_texcoord;

            uniform vec3 sun_direction;
            uniform vec3 A;
            uniform vec3 B;
            uniform vec3 C;
            uniform vec3 D;
            uniform vec3 E;
            uniform vec3 F;
            uniform vec3 G;
            uniform vec3 H;
            uniform vec3 I;
            uniform vec3 Z;
            uniform float N;
            uniform bool to_hdr;

            out vec4 out_color;

            vec3 HosekWilkie(float cos_theta, float gamma, float cos_gamma) {
                vec3 chi = (1 + cos_gamma * cos_gamma) /
                           pow(1 + H * H - 2 * cos_gamma * H, vec3(1.5));
                return (1 + A * exp(B / (cos_theta + 0.01))) *
                       (C + D * exp(E * gamma) + F * (cos_gamma * cos_gamma) +
                        G * chi + I * sqrt(cos_theta));
            }

            vec3 SkyRGB(vec3 v, vec3 sun_dir) {
                vec3 v1 = normalize(v);

                float cos_theta = clamp(v1.z, 0, 1);
                float cos_gamma = clamp(dot(v1, sun_dir), 0, 1);
                float gamma = acos(cos_gamma);

                return Z * HosekWilkie(cos_theta, gamma, cos_gamma) * N;
            }

            void main() {
                vec3 color = SkyRGB(sky_texcoord, sun_direction);
                if (to_hdr) {
                    color *= 2.0;
                    color = pow(color, vec3(2.2));
                }
                out_color = vec4(color, 1);
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Hosek to get internal radiance.
     * (1 + A e ^ (B / cos(t))) (1 + C e ^ (D g) + E cos(g) ^ 2 + F mieM(g, G) +
     *  H cos(t)^1/2 + (I - 1))
     *
     * A: sky gradient, carries white -> blue gradient.
     * B: sky tightness.
     * C: sun, carries most of sun-centred blue term.
     * D: sun tightness, higher = tighter.
     * E: rosy hue around sun.
     *
     * Hosek-specific:
     * F: mie term, does most of the heavy lifting for sunset glow.
     * G: mie tuning.
     * H: zenith gradient.
     * I: constant term balanced with H.
     *
     * Notes:
     * A/B still carries some of the "blue" base of sky, but much comes from
     *     C/D.
     * C/E minimal effect in sunset situations, carry bulk of sun halo in
     *     sun-overhead.
     * F/G sunset glow, but also takes sun halo from yellowish to white
     *     overhead.
     */
    RVector3D HosekWilkie(double cos_theta, double gamma,
                          double cos_gamma) const {
        RVector3D chi;
        for (int t = 0; t < 3; ++t) {
            const double a = coefficents_[0][t];
            const double b = coefficents_[1][t];
            const double c = coefficents_[2][t];
            const double d = coefficents_[3][t];
            const double e = coefficents_[4][t];
            const double f = coefficents_[5][t];
            const double g = coefficents_[6][t];
            const double h = coefficents_[7][t];
            const double i = coefficents_[8][t];

            const double exp_m = std::exp(e * gamma);
            // Rayleigh scattering.
            const double ray_m = cos_gamma * cos_gamma;
            const double mie_m = (1.0f + ray_m) /
                            std::pow((1.0 + h * h - 2.0 * h * cos_gamma), 1.5);
            // Vertical zenith gradient.
            const double zenith = std::sqrt(std::max(0.0, cos_theta));

            chi[t] = (1.0 + a * std::exp(b / (cos_theta + 0.01))) *
                     (c + d * exp_m + f * ray_m + g * mie_m + i * zenith);
        }
        return chi;
    }

    /**
     * Internal use.
     */
    double EvaluateSpline(const double* spline, int stride, double value) {
        double s1 = value;
        double s2 = s1 * s1;
        double s3 = s1 * s2;
        double s4 = s1 * s3;
        double s5 = s1 * s4;

        double is1 = (1.0 - value);
        double is2 = is1 * is1;
        double is3 = is1 * is2;
        double is4 = is1 * is3;
        double is5 = is1 * is4;

        return 1.0  * is5      * spline[0 * stride] +
               5.0  * is4 * s1 * spline[1 * stride] +
               10.0 * is3 * s2 * spline[2 * stride] +
               10.0 * is2 * s3 * spline[3 * stride] +
               5.0  * is1 * s4 * spline[4 * stride] +
               1.0  *       s5 * spline[5 * stride];
    }

    /**
     * Evaluate the coefficient.
     */
    float Evaluate(const double* dataset, int stride) {
        // Splines are functions of elevation^1/3.
        double elevation_k =
                std::pow(std::max(0.0, 1.0 - 2.0 * sun_theta_ / numbers::pi),
                         1.0 / 3.0);

        // Table has values for turbidity 1..10.
        int turbidity0 = Clamp(static_cast<int>(turbidity_), 1, 10);
        int turbidity1 = std::min(turbidity0 + 1, 10);
        double turbidity_k = Clamp(turbidity_ - turbidity0, 0.0, 1.0);

        const double* dataset_a0 = dataset;
        const double* dataset_a1 = dataset + stride * 6 * 10;

        double a0t0 = EvaluateSpline(dataset_a0 + stride * 6 * (turbidity0 - 1),
                                     stride, elevation_k);
        double a1t0 = EvaluateSpline(dataset_a1 + stride * 6 * (turbidity0 - 1),
                                     stride, elevation_k);
        double a0t1 = EvaluateSpline(dataset_a0 + stride * 6 * (turbidity1 - 1),
                                     stride, elevation_k);
        double a1t1 = EvaluateSpline(dataset_a1 + stride * 6 * (turbidity1 - 1),
                                     stride, elevation_k);

        return static_cast<float>(a0t0 * (1.0 - albedo_) * (1.0 - turbidity_k) +
                                  a1t0 *        albedo_  * (1.0 - turbidity_k) +
                                  a0t1 * (1.0 - albedo_) * turbidity_k +
                                  a1t1 *        albedo_  * turbidity_k);
    }

    /**
     * Convert the sky model to SDR or HDR cubemap.
     */
    void ToCubemap(int resolution, bool to_hdr, Cubemap* cubemap) {
        CHECK(resolution > 0);
        CHECK(cubemap);

        if (!shader_) this->InitializeShader();
        cubemap->Reset(resolution, true);
        this->SetParameters(to_hdr);
        cubemap->SetTexture(shader_, true);
    }

    /**
     * Update sun light and ambient light.
     */
    void UpdateLight() {
        sun_light_.set_direction(-sun_position_);
        sun_light_.set_radiance(this->GetSunRadiance());
        ambient_light_.set_radiance(this->GetAmbientColor());
    }

    // Normalized sun position.
    RVector3D sun_position_;

    // Sun theta = acos(sun_position.z)
    double sun_theta_;

    // Coefficents for rendering.
    RVector3D coefficents_[10];

    // The coefficient to normalize the RGB color.
    double rgb_normalize_;

    // Albedo of atmosphere.
    double albedo_ = 0.1;

    // The reduction in the transparency of air because of scattering of light
    // by dust and smoke particles and water droplets.
    // Increase the turbidity to make the atmosphere denser - hazier.
    double turbidity_ = 4.0;

    // The resolution of the cubemap.
    int cubemap_resolution_ = 0;

    // Sky sphere.
    RenderObject sky_sphere_;

    // Sun light.
    Light sun_light_;

    // Ambient light.
    Light ambient_light_;

    // Sky cubemap.
    Cubemap cubemap_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_SKY_H_
