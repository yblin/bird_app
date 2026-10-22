//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_IO_PLY_IO_H_
#define CODELIBRARY_POINT_CLOUD_IO_PLY_IO_H_

#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/mesh/ply_file.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace point_cloud {

/**
 * Read point cloud from PLY file.
 */
class PLYLoader : public geometry::PLYFile {
public:
    PLYLoader() = default;

    explicit PLYLoader(const std::string& filename)
        : PLYFile(filename) {}

    PLYLoader(const PLYLoader&) = delete;

    PLYLoader& operator=(const PLYLoader&) = delete;

    /**
     * Load point cloud data from PLY file.
     */
    template <typename T>
    bool Load(Array<Point3D<T>>* points,
              Array<RGB32Color>* colors = nullptr,
              Array<Vector3D<T>>* normals = nullptr) {
        static_assert(std::is_floating_point<T>::value, "");

        CHECK(points);
        if (!this->is_open()) {
            LOG(ERROR) << "No open file for loading. Forget to call Open()?";
            return false;
        }

        this->LoadElements();

        points->clear();
        if (colors) colors->clear();
        if (normals) normals->clear();

        auto vertex_element = this->GetElement("vertex");
        if (!vertex_element) {
            LOG(ERROR) << "Cannot find element 'vertex'.";
            return false;
        }

        if (!vertex_element->ExtractPoints(points)) return false;
        if (colors) vertex_element->ExtractColors(colors);
        if (normals) vertex_element->ExtractNormals(normals);

        return true;
    }
};

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_IO_PLY_IO_H_
