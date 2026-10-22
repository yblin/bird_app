//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_
#define CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_

#include <string>

#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/util/io/line_reader.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace geometry {

/**
 * Load OBJ format file.
 */
class OBJLoader {
public:
    OBJLoader() = default;

    explicit OBJLoader(const std::string& filename) {
        Open(filename);
    }

    OBJLoader(const OBJLoader&) = delete;

    OBJLoader& operator=(const OBJLoader&) = delete;

    /**
     * Open file for loading.
     */
    bool Open(const std::string& filename) {
        return line_reader_.Open(filename);
    }

    /**
     * Check if a OBJ file is open for loading.
     */
    bool is_open() const {
        return line_reader_.is_open();
    }

    /**
     * Close the open file.
     */
    void Close() {
        line_reader_.Close();
    }

    /**
     * Load OBJ into surface mesh.
     *
     * Return false if error occurs.
     */
    template <typename T>
    bool Load(SurfaceMesh<Point3D<T>>* mesh, bool swap_y_and_z = false) {
        static_assert(std::is_floating_point<T>::value, "");

        CHECK(mesh);

        mesh->clear();

        if (!is_open()) {
            LOG(INFO) << "No open file for loading. Forget to call Open()?";
            return false;
        }

        Array<std::string> elements;
        Array<std::string> values;
        double x, y, z;
        int id;

        using Vertex = typename SurfaceMesh<Point3D<T>>::Vertex;
        Array<int> indices;
        Array<Vertex*> vertices;
        Point3D<T> p;

        char* line = nullptr;
        while ((line = line_reader_.ReadLine())) {
            if (line[0] == '#') continue;
            if (line[0] == 'v' && line[1] == ' ') {
                TextStream stream(line + 2);
                if (!stream.ReadFloat(&x) || !stream.ReadFloat(&y) ||
                    !stream.ReadFloat(&z)) {
                    LOG(ERROR) << ErrorMessage("");
                    mesh->clear();
                    return false;
                }

                if (swap_y_and_z) {
                    p = Point3D<T>(static_cast<T>(x), static_cast<T>(z),
                                   static_cast<T>(y));
                } else {
                    p = Point3D<T>(static_cast<T>(x), static_cast<T>(y),
                                   static_cast<T>(z));
                }
                vertices.push_back(mesh->AddVertex(p));
            } else if (line[0] == 'f' && line[1] == ' ') {
                StringSplit(line, ' ', &elements);
                indices.clear();
                for (int i = 1; i < elements.size(); ++i) {
                    if (elements[i].empty()) continue;
                    StringSplit(elements[i], '/', &values);
                    if (values.empty()) {
                        LOG(ERROR) << ErrorMessage("");
                        mesh->clear();
                        return false;
                    }

                    TextStream stream(values[0]);
                    if (stream.ReadInteger(&id)) {
                        if (id <= 0) {
                            LOG(ERROR) << ErrorMessage("Index must be greater "
                                                       "than 1.");
                            mesh->clear();
                            return false;
                        }
                        indices.push_back(id);
                    }
                }

                if (indices.size() < 3) {
                    LOG(ERROR) << ErrorMessage("The number of indices of the "
                                               "face should not smaller than "
                                               "3.");
                    mesh->clear();
                    return false;
                }

                if (swap_y_and_z) std::reverse(indices.begin(), indices.end());

                Array<Vertex*> face(indices.size());
                for (int i = 0; i < indices.size(); ++i) {
                    if (indices[i] > vertices.size()) {
                        LOG(ERROR) << ErrorMessage("Invalid index detected.");
                        mesh->clear();
                        return false;
                    }
                    face[i] = vertices[indices[i] - 1];
                }

                Vector3D<T> v1 = face[1]->point() - face[0]->point();
                Vector3D<T> v2 = face[2]->point() - face[0]->point();
                Vector3D<T> normal = v1.cross(v2).normalized();
                mesh->AddFace(face, normal);
            }
        }

        return !mesh->empty();
    }

private:
    /**
     * Return common error message.
     */
    std::string ErrorMessage(const std::string& message) const {
        return Format("Invalid OBJ format at line {}. {}",
                      line_reader_.n_line(), message);
    }

    io::LineReader line_reader_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_
