//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_
#define CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_

#include <cfloat>

#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/range_ray_3d.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

class CollisionShape {
public:
    CollisionShape() = default;

    virtual ~CollisionShape() = default;

    /**
     * Pick the collision shape by ray.
     *
     * It also optionally computes the intersection point. If there are multiple
     * intersection points, only the nearest one will saved.
     *
     * Return true if the ray intersects with the collider.
     */
    virtual bool Pick(const RRay3D& ray, RPoint3D* point) const = 0;
};

class BoxCollision : public CollisionShape {
public:
    explicit BoxCollision(const RBox3D& box)
        : box_(box) {}

    bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const override {
        RRangeRay3D rray(ray);
        if (!rray.Intersect(box_) || rray.upper() < 0.0) return false;

        if (point) {
            if (rray.lower() < 0.0)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    RBox3D box_;
};

class SphereCollision : public CollisionShape {
public:
    explicit SphereCollision(const RSphere3D& sphere)
        : sphere_(sphere) {}

    bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const override {
        RRangeRay3D rray(ray);
        if (!rray.Intersect(sphere_) || rray.upper() < 0.0) return false;

        if (point) {
            if (rray.lower() < 0.0)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    RSphere3D sphere_;
};

class CylinderCollision : public CollisionShape {
public:
    explicit CylinderCollision(const RCylinder3D& cylinder)
        : cylinder_(cylinder) {}

    bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const override {
        RRangeRay3D rray(ray);
        if (!rray.Intersect(cylinder_) || rray.upper() < 0.0) return false;

        if (point) {
            if (rray.lower() < 0.0)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    RCylinder3D cylinder_;
};

class ConeCollision : public CollisionShape {
public:
    explicit ConeCollision(const RCone3D& cone)
        : cone_(cone) {}

    bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const override {
        RRangeRay3D rray(ray);
        if (!rray.Intersect(cone_) || rray.upper() < 0.0) return false;

        if (point) {
            if (rray.lower() < 0.0)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    RCone3D cone_;
};

class MeshCollision : public CollisionShape {
public:
    explicit MeshCollision(const Array<RTriangle3D>& mesh)
        : mesh_(mesh) {}

    explicit MeshCollision(const RenderData& mesh) {
        CHECK(mesh.type == GL_TRIANGLES);
        CHECK(mesh.indices.size() % 3 == 0);

        mesh_.reserve(mesh.indices.size() / 3);

        for (int i = 0; i < mesh.indices.size(); i += 3) {
            int a = mesh.indices[i];
            int b = mesh.indices[i + 1];
            int c = mesh.indices[i + 2];
            mesh_.emplace_back(RPoint3D{mesh.vertices[a].x, mesh.vertices[a].y,
                                        mesh.vertices[a].z},
                               RPoint3D{mesh.vertices[b].x, mesh.vertices[b].y,
                                        mesh.vertices[b].z},
                               RPoint3D{mesh.vertices[c].x, mesh.vertices[c].y,
                                        mesh.vertices[c].z});
        }
    }

    bool Pick(const RRay3D& ray, RPoint3D* point = nullptr) const override {
        RRangeRay3D rray(ray);

        double distance = DBL_MAX;
        RPoint3D p;
        for (const RTriangle3D& triangle : mesh_) {
            if (!point) {
                if (rray.Hit(triangle)) return true;
            } else {
                if (rray.Hit(triangle, &p)) {
                    double dis = Distance(p, ray.origin());
                    if (dis < distance) {
                        distance = dis;
                        *point = p;
                    }
                }
            }
        }

        return (distance < DBL_MAX);
    }

protected:
    Array<RTriangle3D> mesh_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_
