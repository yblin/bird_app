//
// Copyright 2020-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_LIGHT_H_
#define CODELIBRARY_WORLD_LIGHT_LIGHT_H_

#include <string>

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/world/node.h"

namespace cl {
namespace world {

/**
 * Light data.
 */
class Light : public Node {
public:
    // Light type.
    enum Type {
        DIRECTIONAL = 0, // Directional light.
        POINT = 1,       // Point light.
        AMBIENT = 2      // Ambient light.
    };

    explicit Light(const std::string& name = "")
        : Node(name) {
        render_mode_ = RenderMode::NO_RENDER;
    }

    Light(const std::string& name, Type type, const RPoint3D& position,
          const RVector3D& direction, const RVector3D& radiance)
        : Node(name),
          type_(type),
          position_(position),
          direction_(direction),
          radiance_(radiance) {
        render_mode_ = RenderMode::NO_RENDER;
    }

    Type type()                  const { return type_;      }
    const RPoint3D& position()   const { return position_;  }
    const RVector3D& direction() const { return direction_; }
    const RVector3D& radiance()  const { return radiance_;  }

    void set_type(Type type) {
        type_ = type;
    }

    void set_position(const RPoint3D& position) {
        position_ = position;
    }

    void set_direction(const RVector3D& direction) {
        direction_ = direction;
    }

    void set_radiance(const RVector3D& radiance) {
        radiance_ = radiance;
    }

private:
    // Type of this light.
    Type type_;

    // Light position. Used for point light.
    RPoint3D  position_;

    // Light direction. Used for directional light.
    RVector3D direction_;

    // Light radiance.
    RVector3D radiance_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_LIGHT_LIGHT_H_
