//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_LIGHT_FACTORY_H_
#define CODELIBRARY_WORLD_LIGHT_LIGHT_FACTORY_H_

#include <string>
#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/world/light/light.h"

namespace cl {
namespace world {

/**
 * Factory to create and manage the life of lights.
 */
class LightFactory {
    LightFactory() = default;

public:
    static LightFactory* GetInstance() {
        static LightFactory factory;
        return &factory;
    }

    /**
     * Create a directional light.
     */
    Light* CreateDirectionalLight(const std::string& name,
                                  const RVector3D& direction,
                                  const RVector3D& radiance) {
        return CreateLight(name,
                           "DirectionalLight",
                           Light::DIRECTIONAL,
                           RPoint3D(0.0, 0.0, 0.0),
                           direction,
                           radiance);
    }
    Light* CreateDirectionalLight(const RVector3D& direction,
                                  const RVector3D& radiance) {
        return CreateDirectionalLight("", direction, radiance);
    }

    /**
     * Create a point light.
     */
    Light* CreatePointLight(const std::string& name,
                            const RPoint3D& position,
                            const RVector3D& radiance) {
        return CreateLight(name,
                           "PointLight",
                           Light::POINT,
                           position,
                           RVector3D(0.0, 0.0, 0.0),
                           radiance);
    }
    Light* CreatePointLight(const RPoint3D& position,
                            const RVector3D& radiance) {
        return CreatePointLight("", position, radiance);
    }

    /**
     * Create an ambient light.
     */
    Light* CreateAmbientLight(const std::string& name,
                              const RVector3D& radiance) {
        return CreateLight(name,
                           "AmbientLight",
                           Light::AMBIENT,
                           RPoint3D(0.0, 0.0, 0.0),
                           RVector3D(0.0, 0.0, 0.0),
                           radiance);
    }
    Light* CreateAmbientLight(const RVector3D& radiance) {
        return CreateAmbientLight("", radiance);
    }

protected:
    /**
     * Create a light.
     */
    Light* CreateLight(const std::string& name,
                       const std::string& type,
                       Light::Type light_type,
                       const RPoint3D& position,
                       const RVector3D& direction,
                       const RVector3D& radiance) {
        Light* light = lights_.Allocate();
        light->set_name(GetID(name, type));
        light->set_type(light_type);
        light->set_position(position);
        light->set_direction(direction);
        light->set_radiance(radiance);

        return light;
    }

    /**
     * Automatic create names for different types.
     */
    std::string GetID(const std::string& name, const std::string& type) {
        return name.empty() ? type + ToString(counts_[type]++) : name;
    }

    // Lights.
    Pool<Light> lights_;

    // Count for each light type.
    std::unordered_map<std::string, int> counts_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_LIGHT_LIGHT_FACTORY_H_
