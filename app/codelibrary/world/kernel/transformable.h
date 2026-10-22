//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_
#define CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_

#include <algorithm>

#include "codelibrary/geometry/angle.h"
#include "codelibrary/opengl/transform.h"

namespace cl {
namespace world {

/**
 * Class to implement transformation. It implements translation, rotation, and
 * scaling operations.
 */
struct Transformable {
    using Transform = gl::RTransform;

public:
    Transformable() = default;

    /**
     * Translate this node.
     */
    bool Translate(const RVector3D& v) {
        return this->Translate(v.x, v.y, v.z);
    }
    bool Translate(double x, double y, double z) {
        RVector3D v = transform_.GetTranslation();
        return this->SetTranslation(v.x + x, v.y + y, v.z + z);
    }

    /**
     * Get translation vector.
     */
    RVector3D GetTranslation() const {
        return transform_.GetTranslation();
    }

    /**
     * Set translation vector.
     */
    bool SetTranslation(const RVector3D& v) {
        return this->SetTranslation(v.x, v.y, v.z);
    }
    bool SetTranslation(double x, double y, double z) {
        if (x != translation_x() || y != translation_y() ||
            z != translation_z()) {
            is_transformed_ = true;
            transform_.SetTranslation(x, y, z);
            return true;
        }

        return false;
    }

    /**
     * Counterclockwise rotation about the local X-axis.
     */
    bool Pitch(double degree) {
        if (degree == 0.0) return false;

        is_transformed_ = true;
        pitch_ += degree;
        transform_.Pitch(DegreeToRadian(degree));
        return true;
    }

    /**
     * Counterclockwise rotation about the local Y-axis.
     */
    bool Yaw(double degree) {
        if (degree == 0.0) return false;

        is_transformed_ = true;
        yaw_ += degree;
        transform_.Yaw(DegreeToRadian(degree));
        return true;
    }

    /**
     * Counterclockwise rotation about the local Z-axis.
     */
    bool Roll(double degree) {
        if (degree == 0.0) return false;

        is_transformed_ = true;
        roll_ += degree;
        transform_.Roll(DegreeToRadian(degree));
        return true;
    }

    /**
     * Get rotation matrix.
     */
    void GetRotation(RMatrix3* rot) const {
        CHECK(rot);

        transform_.GetRotation(rot);
    }

    /**
     * Get rotation quaternion.
     */
    RQuaternion GetRotation() const {
        return transform_.GetRotation();
    }

    /**
     * Rotate this node by quaternion.
     *
     * Note that it dose not set the translation.
     */
    bool Rotate(const RQuaternion& rot) {
        if (rot == RQuaternion::Identity()) return false;

        Transform transform(rot);
        transform_ = transform_ * transform;
        transform_.GetRotation(&pitch_, &yaw_, &roll_);

        return true;
    }

    /**
     * Rotate from vector v1 to v2.
     */
    bool Rotate(const RVector3D& v1, const RVector3D& v2) {
        RQuaternion rot(v1, v2);
        return Rotate(rot);
    }

    /**
     * Set the transformation parameters, including translation, rotation, and
     * scaling.
     *
     * Rotation angles (pitch, yaw, roll) are specified in degrees.
     */
    void SetTransform(double x, double y, double z,
                      double pitch, double yaw, double roll,
                      double scale_x, double scale_y, double scale_z) {
        is_transformed_ = true;
        pitch_ = pitch;
        yaw_   = yaw;
        roll_  = roll;
        scale_x_ = scale_x;
        scale_y_ = scale_y;
        scale_z_ = scale_z;
        transform_.SetTranslation(x, y, z);
        transform_.SetRotation(DegreeToRadian(pitch), DegreeToRadian(yaw),
                               DegreeToRadian(roll));
    }

    /**
     * Set transform.
     */
    void SetTransform(const Transform& transform, double scale = 1.0) {
        CHECK(scale >= 0.0);

        transform.GetRotation(&pitch_, &yaw_, &roll_);

        scale_x_ = scale;
        scale_y_ = scale;
        scale_z_ = scale;
        transform_.SetTranslation(transform.GetTranslation());
        transform_.SetRotation(pitch_, yaw_, roll_);

        pitch_ = RadianToDegree(pitch_);
        yaw_ = RadianToDegree(yaw_);
        roll_ = RadianToDegree(roll_);

        is_transformed_ = true;
    }

    /**
     * Set transform.
     */
    void SetTransform(const Transform& transform, double scale_x,
                      double scale_y, double scale_z) {
        CHECK(scale_x >= 0.0);
        CHECK(scale_y >= 0.0);
        CHECK(scale_z >= 0.0);

        transform.GetRotation(&pitch_, &yaw_, &roll_);

        scale_x_ = scale_x;
        scale_y_ = scale_y;
        scale_z_ = scale_z;
        transform_.SetTranslation(transform.GetTranslation());
        transform_.SetRotation(pitch_, yaw_, roll_);

        pitch_ = RadianToDegree(pitch_);
        yaw_ = RadianToDegree(yaw_);
        roll_ = RadianToDegree(roll_);

        is_transformed_ = true;
    }

    /**
     * Reset the transform to unit.
     */
    void ResetTransform() {
        transform_ = Transform();
        scale_x_ = 1.0;
        scale_y_ = 1.0;
        scale_z_ = 1.0;
        pitch_ = 0.0;
        yaw_   = 0.0;
        roll_  = 0.0;
        is_transformed_ = true;
    }

    /**
     * Get the transform of this node.
     */
    Transform GetTransform() const {
        Transform t = transform_;
        t.Scale(scale_x_, scale_y_, scale_z_);
        return t;
    }

    /**
     * Updates the rotation angles (pitch, yaw, roll) in degrees.
     *
     * Returns false if the new values are the same as the current ones.
     */
    bool SetRotation(double pitch, double yaw, double roll) {
        if (pitch_ == pitch && yaw_ == yaw && roll_ == roll)
            return false;

        is_transformed_ = true;
        pitch_ = pitch;
        yaw_   = yaw;
        roll_  = roll;
        transform_.SetRotation(DegreeToRadian(pitch_), DegreeToRadian(yaw_),
                               DegreeToRadian(roll_));
        return true;
    }

    /**
     * Set scaling factor of this node.
     */
    void SetScale(double scale) {
        CHECK(scale >= 0.0);

        if (scale_x_ == scale && scale_y_ == scale && scale_z_ == scale)
            return;

        scale_x_ = scale;
        scale_y_ = scale;
        scale_z_ = scale;
        is_transformed_ = true;
    }
    void SetScale(double scale_x, double scale_y, double scale_z) {
        CHECK(scale_x >= 0.0);
        CHECK(scale_y >= 0.0);
        CHECK(scale_z >= 0.0);

        if (scale_x_ == scale_x && scale_y_ == scale_y && scale_z_ == scale_z)
            return;

        scale_x_ = scale_x;
        scale_y_ = scale_y;
        scale_z_ = scale_z;
        is_transformed_ = true;
    }

    /**
     * Clone transform to node.
     */
    void CloneTransform(Transformable* node) {
        CHECK(node);

        node->is_transformed_ = true;
        node->scale_x_ = scale_x_;
        node->scale_y_ = scale_y_;
        node->scale_z_ = scale_z_;
        node->pitch_ = pitch_;
        node->yaw_ = yaw_;
        node->roll_ = roll_;
        node->transform_ = transform_;
    }

    void set_scale_x(double scale) {
        CHECK(scale >= 0.0);

        if (scale_x_ == scale) return;

        scale_x_ = scale;
        is_transformed_ = true;
    }

    void set_scale_y(double scale) {
        CHECK(scale >= 0.0);

        if (scale_y_ == scale) return;

        scale_y_ = scale;
        is_transformed_ = true;
    }

    void set_scale_z(double scale) {
        CHECK(scale >= 0.0);

        if (scale_z_ == scale) return;

        scale_z_ = scale;
        is_transformed_ = true;
    }

    // O(1) functions to access the members.
    double translation_x() const { return transform_.translation_x(); }
    double translation_y() const { return transform_.translation_y(); }
    double translation_z() const { return transform_.translation_z(); }
    double pitch()         const { return pitch_;                     }
    double yaw()           const { return yaw_;                       }
    double roll()          const { return roll_;                      }
    double scale_x()       const { return scale_x_;                   }
    double scale_y()       const { return scale_y_;                   }
    double scale_z()       const { return scale_z_;                   }
    bool is_transformed()  const { return is_transformed_;            }
    void set_untransformed()     { is_transformed_ = false;           }

protected:
    // Translation and rotation transform.
    // The complete transform equals to:
    //   [scale_x, scale_y, scale_z, 1] * transform_.
    Transform transform_;

    // Three euler angles in degrees.
    double pitch_ = 0.0, yaw_ = 0.0, roll_ = 0.0;

    // Scaling.
    double scale_x_ = 1.0, scale_y_ = 1.0, scale_z_ = 1.0;

    // Node is transformed before rendering frame begins.
    bool is_transformed_ = false;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_
