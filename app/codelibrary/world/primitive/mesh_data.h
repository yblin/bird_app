//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_
#define CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_

#include <string>

#include "codelibrary/geometry/mesh/obj_io.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/util/io/file_util.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Mesh render data.
 */
struct MeshData : public RenderData {
    MeshData() : RenderData(GL_TRIANGLES) {}

    /**
     * Load mesh data from SurfaceMesh.
     */
    bool Load(const geometry::SurfaceMesh<FPoint3D>& mesh) {
        if (mesh.empty()) return true;

        mesh.ToTriMesh(&vertices, &indices);
        normals.assign(vertices.size(), FVector3D(0.0f, 0.0f, 0.0f));
        for (auto f : mesh.faces()) {
            for (auto& contour : f->contours()) {
                for (auto e : contour) {
                    int v = e->source()->id();
                    normals[v] += f->normal();
                }
            }
        }

        return true;
    }

    /**
     * Load mesh data from local file.
     */
    bool Load(const std::string& filename) {
        std::string suffix = file_util::GetSuffix(filename);

        geometry::SurfaceMesh<FPoint3D> surface_mesh;
        if (suffix == "obj") {
            geometry::OBJLoader obj;
            if (!obj.Open(filename)) return false;
            if (!obj.Load(&surface_mesh)) return false;
        } else {
            LOG(ERROR) << "Unsupport mesh format: " << filename;
            return false;
        }

        return this->Load(surface_mesh);
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_
