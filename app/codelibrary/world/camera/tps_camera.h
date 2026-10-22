//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_
#define CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_

#include <algorithm>

#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/opengl/camera.h"
#include "codelibrary/ui/key_controller.h"
#include "codelibrary/ui/ui.h"

namespace cl {
namespace world {

/**
 * A third-person styled camera for 3D world.
 */
class TPSCamera : public gl::Camera<double> {
public:
    /**
     * Note that we always fix the up direction to +Z axis.
     */
    TPSCamera(const Point& position, const Point& target,
              bool force_depth_zero_to_one = false)
        : Camera(position, target, Vector(0.0, 0.0, 1.0)) {
        CHECK(position != target);

        z_near_ = 0.1;
        z_far_ = 1000.0;

        Vector v = position_ - target_;
        distance_ = Clamp(Distance(position_, target_), z_near_, z_far_);
        this->ResetPosition(target_ + v.normalized() * distance_);

        min_distance_ = z_near_;
        max_distance_ = z_far_;

        roll_angle_ = std::atan2(v.y, v.x);
        pitch_angle_ = std::asin((position_ - target_).z / distance_);

        force_depth_zero_to_one_ = force_depth_zero_to_one;
        this->SetPerspectiveModel(fov_, 1.0, z_near_, z_far_);
    }

    /**
     * Create a TPS camera with only one parameter.
     */
    explicit TPSCamera(double distance, bool force_depth_zero_to_one = false)
        : TPSCamera(Point(0.0, -distance, distance),
                    Point(0.0, 0.0, 0.0),
                    force_depth_zero_to_one) {}

    virtual ~TPSCamera() = default;

    /**
     * Call it after viewing matrix changed.
     */
    void ViewingChanged() override {
        Vector v = position_ - target_;
        roll_angle_ = std::atan2(v.y, v.x);
        pitch_angle_ = std::asin((position_ - target_).z / distance_);
    }

    /**
     * Transform the camera by UI.
     */
    virtual void Update(const UI& ui) {
        if (!allow_ui_control_) return;

        const double step = 5.0;
        // Mouse wheel.
        this->Zoom(ui.mouse_wheel());

        double w = 2.0 * distance_ / std::cos(DegreeToRadian(fov_ * 0.5));
        double translation_scale = std::max(0.01, w / ui.width());

        // Move the camera.
        if (ui.mouse_pressed(ImGuiMouseButton_Right)) {
            RVector2D v = ui.mouse_delta();
            this->HorizentalRotate(-rotation_sensitive_ * v.x);
            this->VerticalRotate(rotation_sensitive_ * v.y);
        } else if (ui.mouse_pressed(ImGuiMouseButton_Middle)) {
            RVector2D v = ui.mouse_delta();

            this->HorizentalMove(-translation_scale * v.x);
            this->VerticalMove(translation_scale * v.y);
        }

        auto key = ui::KeyController::GetInstance();
        if (key->IsShiftDown()) translation_scale *= 10.0;
        if (!key->IsCtrlDown()) {
            if (key->IsKeyDown(ImGuiKey_A)) {
                this->HorizentalMove(-translation_scale * step);
            }
            if (key->IsKeyDown(ImGuiKey_D)) {
                this->HorizentalMove(translation_scale * step);
            }
            if (key->IsKeyDown(ImGuiKey_S)) {
                this->ForwardMove(-translation_scale * step);
            }
            if (key->IsKeyDown(ImGuiKey_W)) {
                this->ForwardMove(translation_scale * step);
            }
        }
    }

    /**
     * Move the camera along the given vector.
     */
    void Move(const Vector& v) {
        this->position_ += v;
        this->LookAt(this->target() + v);
    }

    /**
     * Rotate the camera horizontally around the target.
     */
    void HorizentalRotate(double radian) {
        if (radian == 0.0) return;

        double pi2 = numbers::pi * 2.0;

        roll_angle_ += radian;
        if (roll_angle_ >  pi2) roll_angle_ -= pi2;
        if (roll_angle_ < -pi2) roll_angle_ += pi2;
        UpdatePosition();
    }

    /**
     * Rotate the camera vertically around the target.
     */
    void VerticalRotate(double radian) {
        if (radian == 0.0) return;

        pitch_angle_ += radian;
        // Make sure that when pitch is out of bounds, screen doesn't get
        // flipped.
        pitch_angle_ = Clamp(pitch_angle_, -0.5 * numbers::pi + 0.1,
                             0.5 * numbers::pi - 0.1);
        UpdatePosition();
    }

    /**
     * Move the camera horizontally relative to the target.
     */
    void HorizentalMove(double distance) {
        if (distance == 0.0) return;

        Vector v = this->direction().cross(this->up_direction());
        this->Move(v.normalized() * distance);
    }

    /**
     * Move the camera vertically relative to the target.
     */
    void VerticalMove(double distance) {
        if (distance == 0.0) return;

        Vector v = this->direction().cross(this->up_direction());
        Vector v1 = v.cross(this->direction());
        this->Move(v1.normalized() * distance);
    }

    /**
     * Move the camera forward.
     */
    void ForwardMove(double distance) {
        if (distance == 0.0) return;

        Vector v = this->direction().cross(this->up_direction());
        Vector v1 = this->up_direction().cross(v);
        this->Move(v1.normalized() * distance);
    }

    /**
     * Zoom in or zoom out the camera.
     */
    void Zoom(double step) {
        if (step == 0.0) return;

        double distance = (max_distance_ - min_distance_) / wheel_steps_ * step;
        distance_ += distance;
        distance_ = Clamp(distance_, min_distance_, max_distance_);
        UpdatePosition();
    }

    /**
     * Zoom in or zoom out the camera while keeping the target pixel unchanged.
     */
    void ZoomAtTarget(double step, const RPoint2D& position) {
        if (step == 0.0) return;

        RRay3D ray = this->Ray(position);
        RVector3D v = step * ray.direction().normalized();
        double distance = v.norm();
        RVector3D dir = this->direction();
        double angle = Radian(ray.direction(), this->direction());
        if (step > 0.0)
            distance_ -= std::cos(angle) * distance;
        else
            distance_ += std::cos(angle) * distance;

        distance_ = Clamp(distance_, min_distance_, max_distance_);
        this->position_ += v;
        this->LookAt(this->position_ + dir.normalized() * distance_);
    }

    /**
     * Change z near value.
     */
    void SetZNear(double z) {
        CHECK(z > 0.0);

        z_near_ = z;
        this->SetPerspectiveModel(fov_, 1.0, z_near_, z_far_);
    }

    /**
     * Return the distance from camera to the target.
     */
    double distance() const {
        return distance_;
    }

    void set_wheel_steps(int wheel_steps) {
        CHECK(wheel_steps > 0);
        wheel_steps_ = wheel_steps;
    }

    void set_min_distance(double d) {
        min_distance_ = std::max(d, z_near_);
    }

    void set_max_distance(double d) {
        max_distance_ = std::min(d, z_far_);
    }

    void allow_ui_control(bool flag) {
        allow_ui_control_ = flag;
    }

    bool allow_ui_control() const {
        return allow_ui_control_;
    }

protected:
    /**
     * Update camera's position according to roll angle and pitch angle.
     */
    void UpdatePosition() {
        double c = std::cos(pitch_angle_);

        Point pos(target_.x + std::cos(roll_angle_) * c * distance_,
                  target_.y + std::sin(roll_angle_) * c * distance_,
                  target_.z + std::sin(pitch_angle_) * distance_);
        this->ResetPosition(pos);
    }

    // Angle for rotation.
    double roll_angle_ = 0.0, pitch_angle_ = 0.0;

    // Distance from camera's position to target.
    double distance_ = 0.0;

    // Minimum and maximum allowed distance from camera's position to target.
    double min_distance_ = 0.0, max_distance_ = DBL_MAX;

    // The ratio between the mouse movement distance and the actual camera
    // rotation angle.
    double rotation_sensitive_ = 0.005;

    // Number of wheels from maximum distance to minimum distance.
    int wheel_steps_ = 100;

    // Camera can be controlled by ui or not.
    bool allow_ui_control_ = true;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_
