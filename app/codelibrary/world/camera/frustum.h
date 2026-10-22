//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_CAMERA_FRUSTUM_H_
#define CODELIBRARY_WORLD_CAMERA_FRUSTUM_H_

#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/opengl/camera.h"

namespace cl {
namespace world {

/**
 * Camera frustum.
 */
class Frustum {
public:
    /**
     * Frustum plane.
     */
    struct Plane {
        double a, b, c, d;
    };

    /**
     * Get camera's frustum.
     */
    template <typename T>
    explicit Frustum(const gl::Camera<T>& camera) {
        Array<Point3D<T>> vertices;
        camera.GetFrustum(&vertices);

        Construct(vertices);
    }

    /**
     * Construct frustum from 8 vertices.
     */
    template <typename T>
    explicit Frustum(const Array<Point3D<T>>& vertices) {
        CHECK(vertices.size() == 8);

        Construct(vertices);
    }

    /**
     * Convert box to frustum.
     */
    template <typename T>
    explicit Frustum(const Box3D<T>& box) {
        Array<RPoint3D> vertices = {
            {box.x_min(), box.y_min(), box.z_min()},
            {box.x_max(), box.y_min(), box.z_min()},
            {box.x_max(), box.y_max(), box.z_min()},
            {box.x_min(), box.y_max(), box.z_min()},
            {box.x_min(), box.y_min(), box.z_max()},
            {box.x_max(), box.y_min(), box.z_max()},
            {box.x_max(), box.y_max(), box.z_max()},
            {box.x_min(), box.y_max(), box.z_max()}
        };
        Construct(vertices);
    }

    /**
     * Check if the point is inside the frustum.
     */
    template <typename T>
    bool Inside(const Point3D<T>& point) const {
        for (const Plane& p : planes_) {
            if (p.a * point.x + p.b * point.y + p.c * point.z + p.d < 0.0) {
                return false;
            }
        }
        return true;
    }

    /**
     * Check if a convex polyhedron can be culled by the frustum.
     *
     * The convex polyhedron is defined by its vertices.
     */
    template <typename T>
    bool Cull(const Array<Point3D<T>>& convex_polyhedron) const {
        CHECK(!convex_polyhedron.empty());

        for (const Plane& p : planes_) {
            bool is_forward = false;
            for (const auto& v : convex_polyhedron) {
                if (p.a * v.x + p.b * v.y + p.c * v.z + p.d > 0.0) {
                    is_forward = true;
                    break;
                }
            }
            if (!is_forward) return true;
        }
        return false;
    }

    /**
     * Clip a line segment.
     */
    template <typename T>
    bool Clip(const Segment3D<T>& seg, Segment3D<T>* clipped_line) const {
        static_assert(std::is_floating_point<T>::value);

        CHECK(clipped_line);

        Point3D<T> a = seg.lower_point(), b = seg.upper_point();
        double dis_a, dis_b;
        for (int i = 0; i < 6; ++i) {
            const Plane& plane = planes_[i];
            dis_a = plane.a * a.x + plane.b * a.y + plane.c * a.z + plane.d;
            dis_b = plane.a * b.x + plane.b * b.y + plane.c * b.z + plane.d;

            if (dis_a < 0.0 && dis_b < 0.0) {
                // Completely outside.
                return false;
            }

            // Both points are inside, no clip.
            if (dis_a > 0.0 && dis_b > 0.0) continue;

            // Clipping.
            Vector3D<T> v = b - a;
            double n = -plane.d - plane.a * a.x - plane.b * a.y - plane.c * a.z;
            double d = plane.a * v.x + plane.b * v.y + plane.c * v.z;
            if (d == 0.0) continue;
            double t = n / d;
            Point3D<T> c = a + static_cast<T>(t) * v;
            if (dis_a < 0.0) {
                a = c;
            } else {
                b = c;
            }
        }
        *clipped_line = Segment3D<T>(a, b);
        return true;
    }

private:
    /**
     * Construct frustum from vertices.
     */
    template <typename T>
    void Construct(const Array<Point3D<T>>& vertices) {
        Array<Plane3D<T>> frustum_planes(6);
        frustum_planes[0] = Plane3D<T>(vertices[0], vertices[2], vertices[1]);
        frustum_planes[1] = Plane3D<T>(vertices[4], vertices[5], vertices[6]);
        frustum_planes[2] = Plane3D<T>(vertices[1], vertices[2], vertices[5]);
        frustum_planes[3] = Plane3D<T>(vertices[0], vertices[4], vertices[3]);
        frustum_planes[4] = Plane3D<T>(vertices[0], vertices[1], vertices[5]);
        frustum_planes[5] = Plane3D<T>(vertices[3], vertices[6], vertices[2]);
        for (int i = 0; i < 6; ++i) {
            const Plane3D<T>& plane = frustum_planes[i];
            planes_[i].a = plane.normal().x;
            planes_[i].b = plane.normal().y;
            planes_[i].c = plane.normal().z;
            planes_[i].d = -planes_[i].a * plane.point().x -
                            planes_[i].b * plane.point().y -
                            planes_[i].c * plane.point().z;
        }
    }

    Plane planes_[6];
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_CAMERA_FRUSTUM_H_
