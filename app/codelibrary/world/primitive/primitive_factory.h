//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_
#define CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_

#include <string>
#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/primitive/cone.h"
#include "codelibrary/world/primitive/cube.h"
#include "codelibrary/world/primitive/cube_sphere.h"
#include "codelibrary/world/primitive/cylinder.h"
#include "codelibrary/world/primitive/mesh_data.h"
#include "codelibrary/world/primitive/pipe.h"
#include "codelibrary/world/primitive/point_data.h"
#include "codelibrary/world/primitive/sphere.h"
#include "codelibrary/util/io/file_util.h"

namespace cl {
namespace world {

/**
 * Factory to manager the life of primitive mesh nodes.
 */
class PrimitiveFactory {
    PrimitiveFactory() = default;

public:
    static PrimitiveFactory* GetInstance() {
        static PrimitiveFactory factory;
        return &factory;
    }

    /**
     * Create a node that contain given render object.
     */
    Node* Create(const std::string& name, RenderObject* object) {
        std::string type;
        switch (object->type()) {
        case GL_POINTS:
            type = "Points";
            break;
        case GL_LINES:
            type = "Lines";
            break;
        case GL_TRIANGLES:
            type = "Mesh";
            break;
        default:
            CHECK(false) << "Unknown type.";
        }

        Node* node = nodes_.Allocate();
        node->set_name(GetID(name, type));
        node->AddRenderObject(object);
        return node;
    }
    Node* Create(RenderObject* object) {
        return Create("", object);
    }

    /**
     * Create a node that contain given render data.
     */
    Node* Create(const std::string& name, const RenderData& data) {
        RenderObject* o = render_objects_.Allocate();
        o->SetRenderData(data);
        return Create(name, o);
    }
    Node* Create(const RenderData& data) {
        return Create("", data);
    }

    /**
     * Create an unit cube.
     */
    Node* CreateCube(const std::string& name = "") {
        Cube cube;
        return CreateNode(name, "Cube", &cube);
    }

    /**
     * Create an unit cube sphere with quality (the value range is [0, 8]).
     */
    Node* CreateCubeSphere(const std::string& name, int subdivision = 4) {
        CubeSphere sphere(subdivision);
        return CreateNode(name, "Cubesphere", &sphere);
    }
    Node* CreateCubeSphere(int subdivision = 4) {
        return CreateCubeSphere("", subdivision);
    }

    /**
     * Create a pipe with n_slices.
     */
    Node* CreatePipe(const std::string& name, const Array<FPoint3D>& points,
                     float radius = 1.0f, int n_slices = 32) {
        Pipe pipe(points, radius, n_slices);
        return CreateNode(name, "Pipe", &pipe);
    }
    Node* CreatePipe(const Array<FPoint3D>& points, float radius = 1.0f,
                     int n_slices = 32) {
        return CreatePipe("", points, radius, n_slices);
    }
    Node* CreatePipe(const std::string& name,
                     const Array<FPoint3D>& points,
                     const Array<float>& radius,
                     int n_slices = 32) {
        CHECK(points.size() == radius.size());

        Pipe pipe(points, radius, n_slices);
        return CreateNode(name, "Pipe", &pipe);
    }
    Node* CreatePipe(const Array<FPoint3D>& points,
                     const Array<float>& radius,
                     int n_slices = 32) {
        CHECK(points.size() == radius.size());

        return CreatePipe("", points, radius, n_slices);
    }

    /**
     * Create a unit sphere with n_slices.
     */
    Node* CreateSphere(const std::string& name, int n_slices = 32) {
        Sphere sphere(n_slices);
        return CreateNode(name, "Sphere", &sphere);
    }
    Node* CreateSphere(int n_slices = 32) {
        return CreateSphere("", n_slices);
    }

    /**
     * Create a unit cone with n_slices.
     */
    Node* CreateCone(const std::string& name, int n_slices = 32) {
        Cone cone(n_slices);
        return CreateNode(name, "Cone", &cone);
    }
    Node* CreateCone(int n_slices = 32) {
        return CreateCone("", n_slices);
    }

    /**
     * Create a unit cylinder with n_slices.
     */
    Node* CreateCylinder(const std::string& name, int n_slices = 32) {
        Cylinder cylinder(n_slices);
        return CreateNode(name, "Cylinder", &cylinder);
    }
    Node* CreateCylinder(int n_slices = 32) {
        return CreateCylinder("", n_slices);
    }

    /**
     * Create mesh from SurfaceMesh.
     */
    Node* CreateMesh(const std::string& name,
                     const geometry::SurfaceMesh<FPoint3D>& surface_mesh) {
        MeshData mesh;
        if (!mesh.Load(surface_mesh)) return nullptr;
        return CreateNode(name, "Mesh", &mesh);
    }
    Node* CreateMesh(const geometry::SurfaceMesh<FPoint3D>& surface_mesh) {
        return CreateMesh("", surface_mesh);
    }

    /**
     * Create a mesh from triangles.
     */
    Node* CreateMesh(const std::string& name,
                     const Array<FTriangle3D>& triangles) {
        geometry::SurfaceMesh<FPoint3D> surface_mesh;
        surface_mesh.FromTriangles(triangles);
        return CreateMesh(name, surface_mesh);
    }
    Node* CreateMesh(const Array<FTriangle3D>& triangles) {
        return CreateMesh("", triangles);
    }

    /**
     * Create mesh from local file.
     */
    Node* CreateMesh(const std::string& filename) {
        MeshData mesh;
        if (!mesh.Load(filename)) return nullptr;
        return CreateNode(file_util::GetBaseName(filename), "Mesh", &mesh);
    }

    /**
     * Create point cloud.
     */
    Node* CreatePointCloud(const std::string& name,
                           const Array<FPoint3D>& points) {
        PointData data(points);
        return CreateNode(name, "Point cloud", &data);
    }
    Node* CreatePointCloud(const Array<FPoint3D>& points) {
        return CreatePointCloud("", points);
    }

protected:
    /**
     * Create a node for render data.
     */
    template <typename Data>
    Node* CreateNode(const std::string& name,
                     const std::string& type,
                     Data* data) {
        RenderObject* object = render_objects_.Allocate();
        object->SwapRenderData(data);

        return CreateNode(name, type, object);
    }

    /**
     * Create a node for a render object.
     */
    Node* CreateNode(const std::string& name,
                     const std::string& type,
                     RenderObject* object) {
        Node* node = nodes_.Allocate();
        node->set_name(GetID(name, type));
        node->AddRenderObject(object);
        return node;
    }

    /**
     * Automatic create names for different types.
     */
    std::string GetID(const std::string& name, const std::string& type) {
        return name.empty() ? type + ToString(counts_[type]++) : name;
    }

    // Allocated nodes.
    Pool<Node> nodes_;

    // Mesh objects.
    Pool<RenderObject> render_objects_;

    // Count for each object type.
    std::unordered_map<std::string, int> counts_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_
