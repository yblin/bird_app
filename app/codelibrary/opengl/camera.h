//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_CAMERA_H_
#define CODELIBRARY_OPENGL_CAMERA_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/ray_3d.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/transform.h"

namespace cl {
namespace gl {

// The type of camera.
enum class CameraProjectionType {
    UNKNOWN,
    PERSPECTIVE,
    ORTHO
};

/**
 * 3D camera for OpenGL.
 *
 * The pipeline:
 *           World Coordinates
 *                   |
 *                   | [ Viewing Matrix ]
 *                   |
 *           Camera Coordinates
 *                   |
 *                   | [ Projection Matrix ]
 *                   |
 *       Normalized Device Coordinates: [-1, 1] * [-1, 1] * [-1, 1]
 *
 * When computing the viewing matrix, we adopt right handed coordinate system.
 */
template <typename T>
class Camera {
    static_assert(std::is_floating_point<T>::value);

public:
    using Point = Point3D<T>;
    using Vector = Vector3D<T>;

    /**
     * Construct a camera.
     *
     * Parameters:
     *  position     - position of the camera.
     *  target       - position of the target (where the camera looks at).
     *  up_direction - the up direction of the camear (the default value is +Y).
     */
    explicit Camera(const Point& position      = Point(0, 0, 1),
                    const Point& target        = Point(0, 0, 0),
                    const Vector& up_direction = Vector(0, 1, 0))
        : position_(position),
          target_(target),
          up_direction_(up_direction) {
        CHECK(target != position);

        this->LookAt(target);
    }

    /**
     * Reset position of the camera. The target point is not changed.
     */
    void ResetPosition(const Point& position) {
        CHECK(target_ != position);

        position_ = position;
        LookAt(target_);
    }

    /**
     * Reset camera.
     */
    void Reset(const Point& position,
               const Point& target,
               const Vector& up_direction = Vector(0, 1, 0)) {
        CHECK(target != position);

        position_ = position;
        target_ = target;
        up_direction_ = up_direction;

        LookAt(target_);
    }

    /**
     * Create a viewing matrix.
     *
     * The viewing matrix maps the target(reference) point to the negative z
     * axis and the camera position(eye point) to the origin.
     *
     * When a typical projection matrix is used, the center of the scene
     * therefore maps to the center of the viewport.
     *
     * Similarly, the direction described by the UP vector projected onto the
     * viewing plane is mapped to the positive y axis so that it points upward
     * in the viewport.
     */
    void LookAt(const Point& target) {
        CHECK(target != position_);

        target_ = target;

        Vector dir = (position_ - target_).normalized();
        Vector right = up_direction_.cross(dir).normalized();
        Vector up = dir.cross(right);

        Vector translate = -position_.ToVector();
        T t1 = right.dot(translate);
        T t2 = up.dot(translate);
        T t3 = dir.dot(translate);
        viewing_  = {
            right.x,  up.x, dir.x, 0,
            right.y,  up.y, dir.y, 0,
            right.z,  up.z, dir.z, 0,
            t1,       t2,   t3,    1
        };
        viewing_changed_ = true;
        ViewingChanged();
    }

    /**
     * Call it after viewing matrix changed.
     */
    virtual void ViewingChanged() {}

    /**
     * Set this camera to perspective model.
     *
     * Parameters:
     *  fov    - the field of view angle (in degrees), in the z direction.
     *  aspect - the ratio of width to height.
     *  z_near - distance to the near clipping plane.
     *  z_far  - distance to the far clipping plane.
     */
    void SetPerspectiveModel(T fov, T aspect, T z_near, T z_far) {
        CHECK(fov > 0 && fov < 180);
        CHECK(aspect > 0);
        CHECK(z_near > 0);
        CHECK(z_near <= z_far);

        projection_type_ = CameraProjectionType::PERSPECTIVE;

        // height = 2 * ht * z_near.
        T ht = std::tan(DegreeToRadian(fov) * T(0.5));
        T wt = aspect * ht;
        fov_    = fov;
        aspect_ = aspect;
        z_near_ = z_near;
        z_far_  = z_far;

        T zl = z_far - z_near;
        if (ht == 0 || wt == 0 || zl == 0) return;

        T fn = force_depth_zero_to_one_ ? z_near_ * z_far_
                                        : 2 * z_near_ * z_far_;
        T k = force_depth_zero_to_one_ ? -z_far_ / zl
                                       : -(z_far_ + z_near_) / zl;
        projection_ = {
            1 / wt, 0,      0,          0,
            0,      1 / ht, 0,          0,
            0,      0,      k,         -1,
            0,      0,      -fn / zl,   0};
        projection_changed_ = true;
    }

    /**
     * Set this camera to perspective model with offset.
     *
     * Parameters:
     *  fov      - the field of view angle (in degrees), in the z direction.
     *  aspect   - the ratio of width to height.
     *  z_near   - distance to the near clipping plane.
     *  z_far    - distance to the far clipping plane.
     *  offset_x - offset in X direction.
     *  offset_y - offset in Y direction.
     */
    void SetPerspectiveModelWithOffset(T fov, T aspect, T z_near, T z_far,
                                       T offset_x, T offset_y) {
        CHECK(fov > 0 && fov < 180);
        CHECK(aspect > 0);
        CHECK(z_near > 0);
        CHECK(z_near <= z_far);

        projection_type_ = CameraProjectionType::PERSPECTIVE;

        // height = 2 * ht * z_near.
        T ht = std::tan(DegreeToRadian(fov) * T(0.5));
        T wt = aspect * ht;
        fov_    = fov;
        aspect_ = aspect;
        z_near_ = z_near;
        z_far_  = z_far;

        T zl = z_far - z_near;
        if (ht == 0 || wt == 0 || zl == 0) return;

        T fn = force_depth_zero_to_one_ ? z_near_ * z_far_
                                        : 2 * z_near_ * z_far_;
        T k = force_depth_zero_to_one_ ? -z_far_ / zl
                                       : -(z_far_ + z_near_) / zl;
        projection_ = {
            1 / wt,    0,         0,          0,
            0,         1 / ht,    0,          0,
            -offset_x, -offset_y, k,         -1,
            0,         0,        -fn / zl,   0};
        projection_changed_ = true;
    }

    /**
     * Set the ortho model.
     *
     * Parameters:
     *  left   - The coordinates for the left vertical clipping plane.
     *  right  - The coordinates for the right vertical clipping plane.
     *  bottom - The coordinates for the bottom horizontal clipping plane.
     *  top    - The coordinates for the top horizontal clipping plans.
     *  z_near - Distance to the near clipping plane.
     *  z_far  - Distance to the far clipping plane.
     */
    void SetOrthoModel(T left, T right, T bottom, T top, T z_near, T z_far) {
        CHECK(left <= right);
        CHECK(bottom <= top);
        CHECK(z_near <= z_far);

        projection_type_ = CameraProjectionType::ORTHO;

        ortho_clipping_plane_ = Box2D<T>(left, right, bottom, top);
        T height   = top - bottom;
        T width    = right - left;
        T zl = z_far - z_near;
        z_near_        = z_near;
        z_far_         = z_far;
        fov_           = 0;
        if (height == 0 || width == 0 || zl == 0) return;

        aspect_ = width / height;

        T w1 = -(right + left) / width;
        T h1 = -(top + bottom) / height;

        T fn = force_depth_zero_to_one_ ? -z_near_
                                        : -(z_far_ + z_near_);
        T k = force_depth_zero_to_one_ ? -2 / zl
                                       : -1 / zl;

        projection_ = {
            2 / width, 0,          0,        0,
            0,         2 / height, 0,        0,
            0,         0,          k,        0,
            w1,        h1,         fn / zl,  1};
        projection_changed_ = true;
    }

    /**
     * Reset camera according to the given viewport.
     */
    void SetViewport(const Box2D<T>& viewport) {
        if (viewport.empty()) return;

        T w = viewport.x_length(), h = viewport.y_length();
        if (w == 0 || h == 0) return;

        if (projection_type_ == CameraProjectionType::PERSPECTIVE) {
            T aspect = w / h;
            if (aspect != aspect_) {
                this->SetPerspectiveModel(fov_, aspect, z_near_, z_far_);
            }
        }
    }

    /**
     * Get NDC coordinate of the given real world coordinate.
     */
    Point Project(const Point& p) const {
        CHECK(projection_type_ != CameraProjectionType::UNKNOWN);

        Vector4D<T> in(p.x, p.y, p.z, 1);
        in = in * viewing_;
        in = in * projection_;
        return Point(in.x / in.w, in.y / in.w, in.z / in.w);
    }

    /**
     * Get the project vector of 'v'.
     */
    Vector Project(const Vector& v) const {
        CHECK(projection_type_ != CameraProjectionType::UNKNOWN);

        Vector4D<T> in(v.x, v.y, v.z, 0);
        in = in * viewing_;
        in.w = 0;

        in = in * projection_;
        return Vector(in.x, in.y, in.z);
    }

    /**
     * Get the unproject point of NDC coordinate.
     *
     * Note that the output point is in 4D homogeneous coordinate.
     */
    Point Unproject(const Point& p) const {
        CHECK(projection_type_ != CameraProjectionType::UNKNOWN);

        Vector4D<T> in(p.x, p.y, p.z, 1);
        if (viewing_changed_) {
            viewing_changed_ = false;
            inverse_viewing_ = viewing_.Inverse();
        }
        if (projection_changed_) {
            projection_changed_ = false;
            inverse_projection_ = projection_.Inverse();
        }
        in = in * inverse_projection_;
        in = in * inverse_viewing_;
        return Point(in.x / in.w, in.y / in.w, in.z / in.w);
    }

    /**
     * Compute the frustum of the camera with given z_near and z_far.
     *
     * The frustum is composed of 8 vertices.
     */
    void GetFrustum(T z_near, T z_far, Array<Point>* vertices) const {
        CHECK(z_near <= z_far);
        CHECK(vertices);
        CHECK(projection_type_ != CameraProjectionType::UNKNOWN)
            << "Need to call SetPerspectiveModel() or SetOrthoModel() first.";

        vertices->resize(8);
        Vector direction = (target_ - position_).normalized();
        Vector right     = direction.cross(up_direction_).normalized();
        Vector up        = right.cross(direction);

        // Near center and far center.
        Point nc = position_ + direction * z_near;
        Point fc = position_ + direction * z_far;

        if (projection_type_ == CameraProjectionType::PERSPECTIVE) {
            // These heights and widths are half the heights and widths of
            // the near and far plane rectangles.
            T fov = DegreeToRadian(fov_);

            T near_height = std::tan(fov / 2) * z_near;
            T near_width  = near_height * aspect_;
            T far_height  = std::tan(fov / 2) * z_far;
            T far_width   = far_height * aspect_;

            // Near plane.
            (*vertices)[0] = nc - right * near_width - up * near_height;
            (*vertices)[1] = nc + right * near_width - up * near_height;
            (*vertices)[2] = nc + right * near_width + up * near_height;
            (*vertices)[3] = nc - right * near_width + up * near_height;
            // Far plane.
            (*vertices)[4] = fc - right * far_width - up * far_height;
            (*vertices)[5] = fc + right * far_width - up * far_height;
            (*vertices)[6] = fc + right * far_width + up * far_height;
            (*vertices)[7] = fc - right * far_width + up * far_height;
        } else {
            T x_min = ortho_clipping_plane_.x_min();
            T x_max = ortho_clipping_plane_.x_max();
            T y_min = ortho_clipping_plane_.y_min();
            T y_max = ortho_clipping_plane_.y_max();

            // Near plane.
            (*vertices)[0] = nc + right * x_min + up * y_min;
            (*vertices)[1] = nc + right * x_max + up * y_min;
            (*vertices)[2] = nc + right * x_max + up * y_max;
            (*vertices)[3] = nc + right * x_min + up * y_max;
            // Far plane.
            (*vertices)[4] = fc + right * x_min + up * y_min;
            (*vertices)[5] = fc + right * x_max + up * y_min;
            (*vertices)[6] = fc + right * x_max + up * y_max;
            (*vertices)[7] = fc + right * x_min + up * y_max;
        }
    }

    /**
     * Compute the frustum of the camera.
     *
     * The frustum is composed of 8 vertices.
     */
    void GetFrustum(Array<Point>* vertices) const {
        GetFrustum(z_near_, z_far_, vertices);
    }

    /**
     * Get screen point from real world point.
     */
    Point2D<T> GetScreenPoint(const Point& p) const {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);

        Point q = this->Project(p);
        return {0.5 * (q.x + 1) * v[2] + v[0], 0.5 * (q.y + 1) * v[3] + v[1]};
    }

    /**
     * Get camera ray through the current screen position.
     */
    Ray3D<T> Ray(const Point2D<T>& pos) const {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);

        // Convert screen coordinate to normalized device coordinate.
        T x = 2 * (pos.x - v[0]) / v[2] - 1;
        T y = 2 * (pos.y - v[1]) / v[3] - 1;

        Point dir = this->Unproject(Point(x, y, -1));
        return Ray3D<T>(position_, dir);
    }

    /**
     * Compute near plane of the camera.
     */
    Plane3D<T> near_plane() const {
        Vector v = Normalize(direction());

        return Plane3D<T>(position_ + z_near_ * v, v);
    }

    /**
     * Compute far plane of the camera.
     */
    Plane3D<T> far_plane() const {
        Vector v = Normalize(direction());

        return Plane3D<T>(position_ + z_far_ * v, v);
    }

    /**
     * Return the projection type of this camera.
     */
    const CameraProjectionType& type() const {
        return projection_type_;
    }

    /**
     * Return the camera's position.
     */
    const Point& position() const {
        return position_;
    }

    /**
     * Return the target that the camera looks at.
     */
    const Point& target() const {
        return target_;
    }

    /**
     * Return the direction from camera's position to target.
     */
    const Vector direction() const {
        return target_ - position_;
    }

    /**
     * Return the up direction of the camera.
     */
    const Vector& up_direction() const {
        return up_direction_;
    }

    /**
     * Return the viewing matrix (world to camera space).
     */
    const Transform<T>& viewing() const {
        return viewing_;
    }

    /**
     * Return the projection matrix (camera space to NDC space).
     */
    const Transform<T>& projection() const {
        return projection_;
    }

    /**
     * The distance to the near clipping plane.
     */
    T z_near() const {
        return z_near_;
    }

    /**
     * The distance to the far clipping plane.
     */
    T z_far() const {
        return z_far_;
    }

    /**
     * Camera fov.
     */
    T fov() const {
        return fov_;
    }

    /**
     * Return the value of width / height.
     */
    T aspect() const {
        return aspect_;
    }

    void set_force_depth_zero_to_one(bool flag) {
        force_depth_zero_to_one_ = flag;
    }

protected:
    // The projection type of the camera.
    CameraProjectionType projection_type_ = CameraProjectionType::UNKNOWN;

    // Camera fov.
    T fov_ = 60;

    // width / height.
    T aspect_ = 0;

    // The distance to the near clipping plane.
    T z_near_ = 0;

    // The distance to the far clipping plane.
    T z_far_ = 0;

    // Clipping plane for ortho camera (in camera coordinate system).
    Box2D<T> ortho_clipping_plane_;

    // The position of the camera (in world coordinate system).
    Point position_;

    // The target point (where the camera looks at).
    Point target_;

    // Up direction of the camera.
    Vector up_direction_;

    // The viewing matrix to transform the world space into camera space.
    Transform<T> viewing_;

    // The projection matrix to transform the camera space into NDC
    // (Normalized Device Coordinates) space.
    Transform<T> projection_;

    // The inverse matrix of viewing matrix.
    mutable Transform<T> inverse_viewing_;

    // The inverse matrix of projection matrix.
    mutable Transform<T> inverse_projection_;

    // Need to update inverse matrices or not.
    mutable bool viewing_changed_ = true;
    mutable bool projection_changed_ = true;

    // The depth of OpenGL camera is -1 to 1.
    // However, the depth of the vulkan camera is 0 to 1. In order to be
    // consistent with it, a parameter is needed to force the depth to be
    // converted to 0 to 1.
    bool force_depth_zero_to_one_ = false;
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_CAMERA_H_
