//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_
#define CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_

#include <memory>

#include "codelibrary/world/kernel/collision_shape.h"

namespace cl {
namespace world {

/**
 * Class to implement collision.
 *
 * It defines the basic collision operations for 'world::Node'.
 */
struct Collidable {
public:
    Collidable() = default;

    Collidable(const Collidable&) = delete;

    Collidable& operator=(const Collidable&) = delete;

    /**
     * Clear the collider.
     */
    void ClearCollider() {
        collision_shapes_.clear();
    }

    /**
     * Add a box collision into collider.
     */
    void AddBoxCollision(const RBox3D& box = RBox3D(-1.0, 1.0, -1.0, 1.0,
                                                    -1.0, 1.0)) {
        collision_shapes_.emplace_back(new BoxCollision(box));
    }

    /**
     * Add a sphere collision into collider.
     */
    void AddSphereCollision(const RSphere3D& sphere =
                            RSphere3D(RPoint3D(0.0, 0.0, 0.0), 1.0)) {
        collision_shapes_.emplace_back(new SphereCollision(sphere));
    }

    /**
     * Add a cylinder collision into collider.
     */
    void AddCylinderCollision(const RCylinder3D& cylinder =
                              RCylinder3D(RPoint3D(0.0, 0.0, -1.0),
                                          RPoint3D(0.0, 0.0,  1.0),
                                          1.0)) {
        collision_shapes_.emplace_back(new CylinderCollision(cylinder));
    }

    /**
     * Add a cone collision into collider.
     */
    void AddConeCollision(const RCone3D& cone =
                          RCone3D(RPoint3D(0.0, 0.0, 1.0),
                                  RPoint3D(0.0, 0.0, 0.0), 1.0)) {
        collision_shapes_.emplace_back(new ConeCollision(cone));
    }

    /**
     * Add a mesh collision into collider.
     */
    void AddMeshCollision(const Array<RTriangle3D>& mesh) {
        collision_shapes_.emplace_back(new MeshCollision(mesh));
    }

    /**
     * Add a mesh collision into collider.
     */
    void AddMeshCollision(const RenderData& mesh) {
        collision_shapes_.emplace_back(new MeshCollision(mesh));
    }

protected:
    // We allow muliple collision shapes.
    Array<std::unique_ptr<CollisionShape>> collision_shapes_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_
