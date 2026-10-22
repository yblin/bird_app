//
// Copyright 2023-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
#define CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_

#include <algorithm>
#include <set>
#include <string>
#include <unordered_map>

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/multi_polygon_3d.h"
#include "codelibrary/geometry/topology/planar_arrangement_3d.h"
#include "codelibrary/geometry/triangle_3d.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/geometry/util/pad_3d.h"
#include "codelibrary/graph/dijkstra_shortest_paths.h"
#include "codelibrary/util/list/indexed_list.h"

namespace cl {
namespace geometry {

/**
 * SurfaceMesh is more general than HalfedgeList<Point3D>. It is designed to
 * hold any general 3D polygonal mesh, but HalfedgeList supports up to two faces
 * per edge.
 */
template <typename Point>
class SurfaceMesh {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value, "");

    using Arrangement2 = Arrangement2D<T>;

public:
    class BaseVertex;
    class BaseEdge;
    class BaseFace;

    using VertexList  = IndexedList<BaseVertex>;
    using EdgeList    = IndexedList<BaseEdge>;
    using FaceList    = IndexedList<BaseFace>;

    using Vertex  = typename VertexList::Node;
    using Edge    = typename EdgeList::Node;
    using Face    = typename FaceList::Node;

    // Base vertex of SurfaceMesh.
    class BaseVertex {
        friend class SurfaceMesh;

    public:
        BaseVertex() = default;

        const Point& point() const { return point_; }

    private:
        // The position of this vertex.
        Point point_;
    };

    // Base edge of SurfaceMesh.
    class BaseEdge {
        friend class SurfaceMesh;

    public:
        BaseEdge() = default;

        Vertex* source()            const { return vertex_;                }
        Vertex* target()            const { return twin_->vertex_;         }
        Edge* twin()                const { return twin_;                  }
        const Point& source_point() const { return vertex_->point_;        }
        const Point& target_point() const { return twin_->vertex_->point_; }

    private:
        // The incident vertex (source vertex).
        Vertex* vertex_ = nullptr;

        // The pointer to the twin edge.
        Edge* twin_ = nullptr;
    };

    // Face of SurfaceMesh.
    // Each face is a polygon with holes. The first contour is the outer contour
    // and other contours are inner holes.
    class BaseFace {
        friend class SurfaceMesh;
    public:
        BaseFace() = default;

        const Array<Array<Edge*>>& contours() const {
            return contours_;
        }

        /**
         * Get the outer contour of the face.
         */
        void GetOutContour(Array<Vertex*>* vertices) {
            CHECK(vertices);

            vertices->clear();
            if (contours_.empty()) return;
            for (auto e : contours_.front()) {
                vertices->push_back(e->source());
            }
        }

        /**
         * Get the plane of the face.
         */
        Plane3D<T> GetPlane() const {
            if (contours_.empty()) return Plane3D<T>();

            CHECK(!contours_[0].empty());
            return Plane3D<T>(contours_[0][0]->source_point(), normal_);
        }

        /**
         * Returns true if the face has a clockwise orientation after being
         * projected onto the specific plane.
         */
        bool IsClockwise(const Plane3D<T>& plane) const {
            CHECK(!contours().empty());

            Pad3D<T> pad(plane);
            Array<Point2D<T>> points;
            for (auto e : contours_.front()) {
                points.push_back(pad.Project(e->source_point()));
            }
            Polygon2D<T> polygon(points);
            return polygon.IsClockwise();
        }

        /**
         * Convert face to polygon.
         */
        void ToPolygon(MultiPolygon3D<T>* polygon) const {
            CHECK(polygon);

            *polygon = MultiPolygon3D<T>(this->GetPlane());
            if (contours_.empty()) return;

            Array<Point> points(contours_[0].size());
            for (int i = 0; i < points.size(); ++i) {
                points[i] = contours_[0][i]->source_point();
            }
            polygon->Insert(points, true);

            for (int i = 1; i < contours_.size(); ++i) {
                points.resize(contours_[i].size());
                for (int j = 0; j < contours_[i].size(); ++j) {
                    points[j] = contours_[i][j]->source_point();
                }
                polygon->Insert(points, false);
            }
        }

        const Vector3D<T>& normal() const {
            return normal_;
        }

        void set_normal(const Vector3D<T>& normal) {
            normal_ = normal;
        }

    private:
        // Normal of face.
        Vector3D<T> normal_;

        // Contours of the face.
        // The first one is outer contour, others are inner contours.
        Array<Array<Edge*>> contours_;
    };

    // Iterators.
    using VertexIterator       = typename VertexList::Iterator;
    using VertexConstIterator  = typename VertexList::ConstIterator;
    using EdgeIterator         = typename EdgeList::Iterator;
    using EdgeConstIterator    = typename EdgeList::ConstIterator;
    using FaceIterator         = typename FaceList::Iterator;
    using FaceConstIterator    = typename FaceList::ConstIterator;

    // Properties.
    template <class T>
    using VertexProperty  = typename VertexList::template Property<T>;
    template <class T>
    using EdgeProperty    = typename EdgeList::template Property<T>;
    template <class T>
    using FaceProperty    = typename FaceList::template Property<T>;

    SurfaceMesh() = default;

    SurfaceMesh(const SurfaceMesh& mesh) = delete;

    SurfaceMesh& operator =(const SurfaceMesh& mesh) = delete;

    /**
     * Clear the data.
     */
    void clear() {
        vertices_.clear();
        edges_.clear();
        faces_.clear();
        vertex_map_.clear();
        vertex_edges_.clear();
        edge_faces_.clear();
    }

    /**
     * Check if the surface mesh is empty.
     */
    bool empty() const {
        return n_vertices() == 0;
    }

    /**
     * Return the number of vertices.
     */
    int n_vertices() const {
        return vertices_.n_available();
    }

    /**
     * Return the number of edges.
     */
    int n_edges() const {
        return edges_.n_available();
    }

    /**
     * Return the number of triangle faces.
     */
    int n_faces() const {
        return faces_.n_available();
    }

    /**
     * Return the number of allocated vertices.
     */
    int n_allocated_vertices() const {
        return vertices_.n_allocated();
    }

    /**
     * Return the number of allocated edges.
     */
    int n_allocated_edges() const {
        return edges_.n_allocated();
    }

    /**
     * Return the number of allocated triangle faces.
     */
    int n_allocated_faces() const {
        return faces_.n_allocated();
    }

    /**
     * Change the position of vertex.
     *
     * Exercise caution when utilizing this function, as it may introduce
     * topological errors in the faces structure.
     * Remember to update the face normals if any changes are made.
     */
    void SetVertexPosition(const Point& p, Vertex* v) {
        CHECK(v);

        auto it = vertex_map_.find(v->point());
        if (it != vertex_map_.end() && it->second == v->id()) {
            vertex_map_.erase(it);
        }

        vertex_map_[p] = v->id();
        v->point_ = p;
    }

    /**
     * Clone this SurfaceMesh.
     *
     * Note that we do NOT clone the properties but just resize them.
     */
    void Clone(SurfaceMesh* mesh) const {
        CHECK(mesh);

        if (this == mesh) return;

        mesh->clear();
        vertices_.Clone(&mesh->vertices_);
        edges_.Clone(&mesh->edges_);
        faces_.Clone(&mesh->faces_);

        auto e1 = edges_.begin();
        auto e2 = mesh->edges_.begin();
        for (; e1 != edges_.end(); ++e1, ++e2) {
            e2->vertex_ = mesh->vertices_[e1->vertex_->id()];
            e2->twin_ = mesh->edges_[e1->twin_->id()];
        }

        for (auto f : mesh->faces()) {
            for (auto& contour : f->contours_) {
                for (auto& e : contour) {
                    e = mesh->edges_[e->id()];
                }
            }
        }

        mesh->vertex_map_ = vertex_map_;
        mesh->vertex_edges_ = vertex_edges_;
        mesh->edge_faces_ = edge_faces_;
    }

    /**
     * Find vertex.
     */
    const Vertex* FindVertex(const Point& p) const {
        auto itr = vertex_map_.find(p);
        return itr == vertex_map_.end() ? nullptr : vertices_[itr->second];
    }
    Vertex* FindVertex(const Point& p) {
        auto itr = vertex_map_.find(p);
        return itr == vertex_map_.end() ? nullptr : vertices_[itr->second];
    }

    /**
     * Find edge from source to target.
     */
    const Edge* FindEdge(const Vertex* source, const Vertex* target) const {
        int id = FindEdgeID(source, target);
        return (id == -1) ? nullptr : edges_[id];
    }
    Edge* FindEdge(Vertex* source, Vertex* target) {
        int id = FindEdgeID(source, target);
        return (id == -1) ? nullptr : edges_[id];
    }
    const Edge* FindEdge(const Point& source, const Point& target) const {
        int id = FindEdgeID(source, target);
        return (id == -1) ? nullptr : edges_[id];
    }
    Edge* FindEdge(const Point& source, const Point& target) {
        int id = FindEdgeID(source, target);
        return (id == -1) ? nullptr : edges_[id];
    }

    /**
     * Add vertex of 'p'.
     *
     * Return the pointer to the new or existing vertex.
     */
    Vertex* AddVertex(const Point& p) {
        Vertex* v = FindVertex(p);
        if (v) return v;

        Vertex* v1 = vertices_.Allocate();
        vertex_map_[p] = v1->id();
        v1->point_ = p;
        return v1;
    }

    /**
     * Add edge (source, target).
     */
    Edge* AddEdge(Vertex* source, Vertex* target) {
        CHECK(source);
        CHECK(target);

        if (source == target) return nullptr;

        Edge* e = FindEdge(source, target);
        if (e) return e;

        Edge* e1 = edges_.Allocate();
        Edge* e2 = edges_.Allocate();
        e1->vertex_ = source;
        e2->vertex_ = target;

        e1->twin_ = e2;
        e2->twin_ = e1;

        vertex_edges_[source->id()].push_back(e1->id());
        vertex_edges_[target->id()].push_back(e2->id());

        return e1;
    }
    Edge* AddEdge(const Point& source, const Point& target) {
        if (source == target) return nullptr;

        return AddEdge(AddVertex(source), AddVertex(target));
    }

    /**
     * Add a new face consist of vertices.
     */
    Face* AddFace(const Array<Vertex*>& vertices, const Vector3D<T>& normal) {
        CHECK(vertices.size() >= 3);

        Array<Edge*> edges;
        for (int j = 0; j < vertices.size(); ++j) {
            Vertex* source = vertices[j];
            Vertex* target = j + 1 == vertices.size() ? vertices[0]
                                                      : vertices[j + 1];
            Edge* e = AddEdge(source, target);
            if (e) edges.push_back(e);
        }
        if (edges.size() < 3) return nullptr;

        Face* face = faces_.Allocate();
        face->normal_ = normal;
        face->contours_.clear();
        face->contours_.push_back(edges);
        for (auto e : edges) {
            edge_faces_[e->id()].insert(face->id());
        }

        return face;
    }

    /**
     * Add a new face consisting of several contours.
     *
     * The first input contour should be outer contour, and its clockwise
     * orientation should be different from the other (inner) contours.
     */
    Face* AddFace(const Array<Array<Vertex*>>& contours,
                  const Vector3D<T>& normal) {
        CHECK(!contours.empty());

        Array<Array<Edge*>> contour_edges;
        for (auto& contour : contours) {
            if (contour.size() < 3) return nullptr;

            Array<Edge*> edges;
            for (int i = 0; i < contour.size(); ++i) {
                int next = i + 1 == contour.size() ? 0 : i + 1;
                Vertex* v1 = contour[i];
                Vertex* v2 = contour[next];

                Edge* e = AddEdge(v1, v2);
                if (e)
                    edges.push_back(e);
            }
            if (edges.size() < 3 && contour_edges.empty()) return nullptr;
            if (edges.size() >= 3) {
                contour_edges.push_back(edges);
            }
        }

        Face* face = faces_.Allocate();
        face->normal_ = normal;
        face->contours_ = contour_edges;
        for (auto edges : contour_edges) {
            for (auto e : edges) {
                edge_faces_[e->id()].insert(face->id());
            }
        }

        return face;
    }

    /**
     * Add a new triangle face.
     */
    Face* AddTriangle(const Triangle3D<T>& triangle) {
        CHECK(!triangle.empty());

        Array<Vertex*> vertices(3);
        for (int i = 0; i < 3; ++i) {
            vertices[i] = this->AddVertex(triangle.vertices()[i]);
        }
        Vector3D<T> normal = (vertices[1]->point() -
                              vertices[0]->point()).cross(
                              vertices[2]->point() -
                              vertices[0]->point()).normalized();
        return this->AddFace(vertices, normal);
    }

    /**
     * Erase vertex.
     */
    void EraseVertex(Vertex* v) {
        CHECK(v);

        auto edges = vertex_edges_[v->id()];
        for (auto eid : edges) {
            Edge* e = edges_[eid];
            EraseEdge(e);
        }
        vertex_edges_.erase(v->id());
        vertex_map_.erase(v->point());

        vertices_.Deallocate(v);
    }

    /**
     * Erase an edge.
     */
    void EraseEdge(Edge* e) {
        CHECK(e);

        EraseFaces(e);
        EraseFaces(e->twin());

        vertex_edges_[e->source()->id()].erase(e->id());
        vertex_edges_[e->target()->id()].erase(e->twin()->id());

        edges_.Deallocate(e);
        edges_.Deallocate(e->twin());
    }

    /**
     * Erase the incident faces of 'e'.
     */
    void EraseFaces(Edge* e) {
        for (auto fid : edge_faces_[e->id()]) {
            Face* f = faces_[fid];
            auto& out_contour = f->contours_[0];

            bool on_out_contour = false;
            for (auto e1 : out_contour) {
                if (e1 == e) {
                    on_out_contour = true;
                    break;
                }
            }
            if (on_out_contour) {
                EraseFace(f);
            } else {
                EraseFaceInnerContour(f, e);
            }
        }
        edge_faces_.erase(e->id());
    }

    /**
     * Erase a face.
     */
    void EraseFace(Face* f) {
        CHECK(f);

        for (auto& contour : f->contours_) {
            for (auto e : contour) {
                edge_faces_[e->id()].erase(f->id());
                if (edge_faces_[e->id()].empty()) {
                    edge_faces_.erase(e->id());
                }
            }
        }
        faces_.Deallocate(f);
    }

    /**
     * Erase a inner contour of the face.
     */
    void EraseFaceInnerContour(Face* f, Edge* e) {
        CHECK(f);
        CHECK(e);

        Array<Array<Edge*>> contours;
        int n_contours = 1;
        for (int i = 1; i < f->contours_.size(); ++i) {
            bool on_contour = false;
            for (auto e1 : f->contours_[i]) {
                if (e1 == e) {
                    on_contour = true;
                    break;
                }
            }
            if (on_contour) {
                for (auto e1 : f->contours_[i]) {
                    edge_faces_[e1->id()].erase(f->id());
                    if (edge_faces_[e1->id()].empty()) {
                        edge_faces_.erase(e1->id());
                    }
                }
            } else {
                f->contours_[n_contours++] = f->contours_[i];
            }
        }
        f->contours_.resize(n_contours);
    }

    /**
     * Erase all isolated vertices.
     */
    void EraseIsolatedVertices() {
        Array<Vertex*> vertices;
        for (auto v : vertices_) {
            if (is_isolated_vertex(v))
                vertices.push_back(v);
        }
        for (auto v : vertices) {
            EraseVertex(v);
        }
    }

    /**
     * Erase all isolated edges.
     */
    void EraseIsolatedEdges() {
        Array<Edge*> edges;
        for (auto e : edges_) {
            if (is_isolated_edge(e) && e->id() % 2 == 0)
                edges.push_back(e);
        }
        for (auto e : edges) {
            EraseEdge(e);
        }
    }

    /**
     * Check if the vertex is on the face.
     */
    bool VertexOnFace(const Vertex* v, const Face* face) const {
        CHECK(v);
        CHECK(face);

        for (const auto& contour : face->contours()) {
            for (auto e : contour) {
                if (e->source() == v) return true;
            }
        }
        return false;
    }

    /**
     * Check if the edge is on the face.
     */
    bool EdgeOnFace(const Edge* e, const Face* face) const {
        CHECK(e);
        CHECK(face);

        for (const auto& contour : face->contours()) {
            for (auto e1 : contour) {
                if (e1 == e) return true;
            }
        }
        return false;
    }

    /**
     * Replace vertex 'v1' with 'v2' and then delete 'v1'.
     *
     * Use this function with caution, as it will modify all edges and faces
     * associated with 'v1'.
     */
    void ReplaceVertex(Vertex* v1, Vertex* v2) {
        if (v1 == v2) return;

        for (auto eid : vertex_edges_[v1->id()]) {
            auto e = this->edge(eid);
            CHECK(e->source() == v1);

            Edge* e1 = AddEdge(v2, e->target());
            ReplaceEdge(e, e1);

            Edge* e2 = AddEdge(e->target(), v2);
            ReplaceEdge(e->twin(), e2);
        }

        EraseVertex(v1);
    }

    /**
     * Split edge 'e' to two edges: (e->source(), p) and (p, e->target()).
     *
     * Make sure to align 'p' properly on 'e' to avoid creating an invalid face.
     */
    bool SplitEdge(Edge* e, const Point& p) {
        Vertex* v = this->AddVertex(p);
        if (v == e->source() || v == e->target()) return false;

        Edge* e1 = this->AddEdge(e->source(), v);
        Edge* e2 = this->AddEdge(v, e->target());

        for (int i = 0; i < 2; ++i) {
            Edge* e3 = i == 0 ? e : e->twin();
            Array<Face*> faces;
            GetIncidentFaces(e3, &faces);
            for (auto f : faces) {
                for (auto& contour : f->contours_) {
                    Array<Edge*> new_contour;
                    for (auto e_c : contour) {
                        if (e3 == e_c) {
                            if (i == 0) {
                                new_contour.push_back(e1);
                                new_contour.push_back(e2);
                            } else {
                                new_contour.push_back(e2->twin());
                                new_contour.push_back(e1->twin());
                            }
                        } else {
                            new_contour.push_back(e_c);
                        }
                    }
                    contour = new_contour;
                }
                if (i == 0) {
                    edge_faces_[e2->id()].insert(f->id());
                    edge_faces_[e1->id()].insert(f->id());
                } else {
                    edge_faces_[e2->twin()->id()].insert(f->id());
                    edge_faces_[e1->twin()->id()].insert(f->id());
                }
                edge_faces_[e3->id()].erase(f->id());
            }
        }

        this->EraseEdge(e);
        return true;
    }

    /**
     * Convert the given face to triangles.
     */
    void Triangulate(const Face* face, Array<Array<Vertex*>>* faces) const {
        CHECK(face);
        CHECK(faces);

        faces->clear();
        if (face->contours_.empty()) return;

        if (face->contours_.size() == 1 && face->contours_[0].size() == 3) {
            Array<Vertex*> tri(3);
            for (int i = 0; i < 3; ++i)
                tri[i] = face->contours_[0][i]->source();
            faces->push_back(tri);
            return;
        }

        Plane3D<T> plane(face->contours_[0][0]->source_point(), face->normal_);
        PlanarArrangement3D<T> arrangement(plane);
        std::unordered_map<typename Arrangement2::Vertex*, Vertex*> hash;
        for (auto& contour : face->contours_) {
            for (auto e : contour) {
                auto v = arrangement.Insert(e->source_point());
                hash[v] = e->source();
            }
        }

        bool clockwise = face->IsClockwise(plane);
        for (auto& contour : face->contours_) {
            for (auto e : contour) {
                if (clockwise) {
                    arrangement.InsertWithoutNewVertex(e->source_point(),
                                                       e->target_point(),
                                                       0,
                                                       1);
                } else {
                    arrangement.InsertWithoutNewVertex(e->source_point(),
                                                       e->target_point(),
                                                       1,
                                                       0);
                }
            }
        }

        arrangement.Arrange();
        for (auto region : arrangement.arrangement2d().regions()) {
            if (IsHiddenRegion(arrangement.arrangement2d(), region)) continue;

            Array<Vertex*> vertices(3);
            for (auto f : region) {
                auto e = f->halfedge;
                vertices[0] = hash[e->source()];
                vertices[1] = hash[e->target()];
                vertices[2] = hash[e->next()->target()];
                faces->push_back(vertices);
            }
        }
    }

    /**
     * Find the shortest path from source to target.
     */
    bool FindPath(Vertex* source, Vertex* target, Array<Vertex*>* path) {
        CHECK(source);
        CHECK(target);
        CHECK(path);

        path->clear();
        if (source == target) {
            path->push_back(target);
            return true;
        }

        Graph graph(this->n_allocated_vertices());
        auto weight = graph.AddEdgeProperty("weight", 0.0);
        for (auto e : edges_) {
            if (e->id() % 2 == 1) continue;
            auto e1 = graph.InsertTwoWayEdge(e->source()->id(),
                                            e->target()->id());
            weight[e1] = weight[e1->twin()] = Distance(e->source_point(),
                                                       e->target_point());
        }

        Array<double> distance;
        Array<int> predecessors;
        graph::DijkstraShortestPaths(graph, weight, source->id(), target->id(),
                                     &predecessors, &distance);

        if (predecessors[target->id()] == -1) return false;

        for (Vertex* v = target; ; v = vertices_[predecessors[v->id()]]) {
            path->push_back(v);
            if (v == source) break;
        }
        std::reverse(path->begin(), path->end());

        return true;
    }

    // O(1) functions
    /**
     * Return true if an halfedge has no attached face.
     */
    bool is_isolated_halfedge(const Edge* e) const {
        auto it = edge_faces_.find(e->id());
        return it == edge_faces_.end() || it->second.empty();
    }

    /**
     * Return true if an edge has no attached face.
     */
    bool is_isolated_edge(const Edge* e) const {
        return is_isolated_halfedge(e) && is_isolated_halfedge(e->twin());
    }

    /**
     * Return true if a vertex has no attached edge.
     */
    bool is_isolated_vertex(const Vertex* v) const {
        auto it = vertex_edges_.find(v->id());
        return it == vertex_edges_.end() || it->second.empty();
    }

    // Access functions.
    const Array<Vertex*>& vertices() const { return vertices_.nodes(); }
    const Array<Edge*>& edges()      const { return edges_.nodes();    }
    const Array<Face*>& faces()      const { return faces_.nodes();    }

    Vertex* vertex(int id)               { return vertices_[id];  }
    const Vertex* vertex(int id)   const { return vertices_[id];  }
    Edge* edge(int id)                   { return edges_[id];     }
    const Edge* edge(int id)       const { return edges_[id];     }
    Face* face(int id)                   { return faces_[id];     }
    const Face* face(int id)       const { return faces_[id];     }

    /**
     * Return the attached faces of edge 'e'.
     */
    void GetIncidentFaces(const Edge* e, Array<const Face*>* faces) const {
        CHECK(faces);

        faces->clear();
        auto it = edge_faces_.find(e->id());
        if (it == edge_faces_.end()) return;

        for (auto fid : it->second) {
            faces->push_back(faces_[fid]);
        }
    }
    void GetIncidentFaces(Edge* e, Array<Face*>* faces) {
        CHECK(faces);

        faces->clear();
        auto it = edge_faces_.find(e->id());
        if (it == edge_faces_.end()) return;

        for (auto fid : it->second) {
            faces->push_back(faces_[fid]);
        }
    }

    /**
     * Return the attached faces of edge 'e' and 'e->twin()'.
     */
    void GetUndirectedIncidentFaces(const Edge* e,
                                    Array<const Face*>* faces) const {
        CHECK(faces);

        faces->clear();
        Array<const Face*> fs;
        this->GetIncidentFaces(e, &fs);
        faces->insert(fs);
        this->GetIncidentFaces(e->twin(), &fs);
        faces->insert(fs);
    }
    void GetUndirectedIncidentFaces(Edge* e, Array<Face*>* faces) {
        CHECK(faces);

        faces->clear();
        Array<Face*> fs;
        this->GetIncidentFaces(e, &fs);
        faces->insert(fs);
        this->GetIncidentFaces(e->twin(), &fs);
        faces->insert(fs);
    }

    /**
     * Return the attached faces of vertex 'v'.
     */
    void GetIncidentFaces(const Vertex* v, Array<const Face*>* faces) const {
        CHECK(faces);

        faces->clear();

        Array<const Edge*> edges;
        Array<const Face*> fs;
        this->GetIncidentEdges(v, &edges);
        for (auto e : edges) {
            this->GetIncidentFaces(e, &fs);
            faces->insert(fs);
        }
    }
    void GetIncidentFaces(Vertex* v, Array<Face*>* faces) {
        CHECK(faces);

        faces->clear();

        Array<Edge*> edges;
        Array<Face*> fs;
        this->GetIncidentEdges(v, &edges);
        for (auto e : edges) {
            this->GetIncidentFaces(e, &fs);
            faces->insert(fs);
        }
    }

    /**
     * Return the incident edges of vertex 'v'.
     */
    void GetIncidentEdges(const Vertex* v, Array<const Edge*>* edges) const {
        CHECK(edges);

        edges->clear();
        auto it = vertex_edges_.find(v->id());
        if (it == vertex_edges_.end()) return;

        for (auto eid : it->second) {
            edges->push_back(edges_[eid]);
        }
    }
    void GetIncidentEdges(Vertex* v, Array<Edge*>* edges) {
        CHECK(edges);

        edges->clear();
        auto it = vertex_edges_.find(v->id());
        if (it == vertex_edges_.end()) return;

        for (auto eid : it->second) {
            edges->push_back(edges_[eid]);
        }
    }

    /**
     * Add a vertex property.
     */
    template <typename T>
    VertexProperty<T> AddVertexProperty(const std::string& name,
                                        const T& initial_value = T()) {
        return vertices_.AddProperty(name, initial_value);
    }

    /**
     * Add a const vertex property.
     */
    template <typename T>
    VertexProperty<T> AddVertexProperty(const T& initial_value = T()) const {
        return vertices_.AddProperty(initial_value);
    }

    /**
     * Get a vertex property.
     */
    template <typename T>
    VertexProperty<T> GetVertexProperty(const std::string& name) const {
        return vertices_.template GetProperty<T>(name);
    }

    /**
     * Add a edge property.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const std::string& name,
                                    const T& initial_value = T()) {
        return edges_.AddProperty(name, initial_value);
    }

    /**
     * Add a const edge property.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const T& initial_v = T()) const {
        return edges_.AddProperty(initial_v);
    }

    /**
     * Get a edge property.
     */
    template <typename T>
    EdgeProperty<T> GetEdgeProperty(const std::string& name) const {
        return edges_.template GetProperty<T>(name);
    }

    /**
     * Add a face property.
     */
    template <typename T>
    FaceProperty<T> AddFaceProperty(const std::string& name,
                                    const T& initial_value = T()) {
        return faces_.AddProperty(name, initial_value);
    }

    /**
     * Add a const face property.
     */
    template <typename T>
    FaceProperty<T> AddFaceProperty(const T& initial_v = T()) const {
        return faces_.AddProperty(initial_v);
    }

    /**
     * Get a face property.
     */
    template <typename T>
    FaceProperty<T> GetFaceProperty(const std::string& name) const {
        return faces_.template GetProperty<T>(name);
    }

    /**
     * Erase a vertex property with the given name.
     */
    void EraseVertexProperty(const std::string& name) {
        vertices_.EraseProperty(name);
    }

    /**
     * Erase a edge property with the given name.
     */
    void EraseEdgeProperty(const std::string& name) {
        edges_.EraseProperty(name);
    }

    /**
     * Erase a face property with the given name.
     */
    void EraseFaceProperty(const std::string& name) {
        faces_.EraseProperty(name);
    }

    /**
     * Clear all vertex properties.
     */
    void ClearVertexProperties() {
        vertices_.ClearAllProperties();
    }

    /**
     * Clear all edge properties.
     */
    void ClearEdgeProperties() {
        edges_.ClearAllProperties();
    }

    /**
     * Clear all face properties.
     */
    void ClearFaceProperties() {
        faces_.ClearAllProperties();
    }

    /**
     * Return true if the given vertex is available.
     */
    bool IsAvailable(Vertex* v) const {
        return vertices_.IsAvailable(v);
    }

    /**
     * Return true if the given edge is available.
     */
    bool IsAvailable(Edge* e) const {
        return edges_.IsAvailable(e);
    }

    /**
     * Return true if the given face is available.
     */
    bool IsAvailable(Face* f) const {
        return faces_.IsAvailable(f);
    }

    /**
     * Return the bounding box of the mesh.
     */
    Box3D<T> GetBoundingBox() const {
        Box3D<T> box;
        if (vertices_.empty()) return box;

        for (auto v : vertices_) {
            box.set_x_range(std::min(box.x_min(), v->point_.x),
                            std::max(box.x_max(), v->point_.x));
            box.set_y_range(std::min(box.y_min(), v->point_.y),
                            std::max(box.y_max(), v->point_.y));
            box.set_z_range(std::min(box.z_min(), v->point_.z),
                            std::max(box.z_max(), v->point_.z));
        }
        return box;
    }

    /**
     * Reconstruct SurfaceMesh from triangle sets.
     */
    void FromTriangles(const Array<Triangle3D<T>>& triangles) {
        this->clear();
        if (triangles.empty()) return;

        for (const Triangle3D<T>& tri : triangles) {
            this->AddTriangle(tri);
        }
    }

    /**
     * Convert SurfaceMesh to compressed tri-mesh for easy transfering and
     * rendering.
     *
     * The compressed tri-mesh consists of a set of vertices and a set of
     * triangles' indices.
     */
    void ToTriMesh(Array<Point>* vertices, Array<int>* indices) const {
        CHECK(vertices);
        CHECK(indices);

        vertices->resize(this->n_allocated_vertices());
        for (auto v : vertices_) {
            (*vertices)[v->id()] = v->point_;
        }

        indices->clear();
        for (auto f : faces_) {
            Array<Array<Vertex*>> triangles;
            this->Triangulate(f, &triangles);
            for (auto tri : triangles) {
                for (auto v : tri)
                    indices->push_back(v->id());
            }
        }
    }

protected:
    /**
     * Return true if the given region is a hidden region.
     */
    static bool IsHiddenRegion(const Arrangement2& arrangement,
                               const typename Arrangement2::Region& region) {
        Array<typename Arrangement2::Halfedge*> edges(3);
        for (auto face : region) {
            auto e = face->halfedge;
            edges[0] = e;
            edges[1] = e->next();
            edges[2] = e->next()->next();
            for (auto e1 : edges) {
                if (arrangement.is_constraint(e1) &&
                    arrangement.color(e1) == 1) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Find edge id of (source, target).
     */
    int FindEdgeID(const Vertex* source, const Vertex* target) const {
        CHECK(source);
        CHECK(target);

        return FindEdgeID(source->id(), target->id());
    }
    int FindEdgeID(int source, int target) const {
        if (source == target) return -1;

        auto itr = vertex_edges_.find(source);
        if (itr == vertex_edges_.end()) return -1;

        for (auto e : itr->second) {
            if (edges_[e]->target()->id() == target) return e;
        }
        return -1;
    }
    int FindEdgeID(const Point& source, const Point& target) const {
        const Vertex* s = this->FindVertex(source);
        if (!s) return -1;
        const Vertex* t = this->FindVertex(target);
        if (!t) return -1;

        return FindEdgeID(s->id(), t->id());
    }

    /**
     * Replace edge 'e1' with 'e2' for all associated edges.
     *
     * The edge 'e1' is preserved.
     *
     * For internal use.
     */
    void ReplaceEdge(Edge* e1, Edge* e2) {
        CHECK(e1);
        CHECK(e1 != e2);

        for (auto fid : edge_faces_[e1->id()]) {
            auto f = this->face(fid);
            for (auto& contour : f->contours_) {
                for (auto& e : contour) {
                    if (e == e1) {
                        if (e2)
                            e = e2;
                        else
                            contour.erase(e);
                        break;
                    }
                }
            }

            if (e2) edge_faces_[e2->id()].insert(fid);
        }
        edge_faces_[e1->id()].clear();
    }

    /**
     * Return reference of contour of the face.
     */
    Array<Array<Edge*>>& GetContours(Face* f) {
        return f->contours_;
    }

    VertexList vertices_;
    EdgeList edges_;
    FaceList faces_;

    // Map from position to vertex.
    std::unordered_map<Point, int> vertex_map_;

    // Incident edges of each vertex.
    // vertex_edges_[v] stores all edges starting from v.
    std::unordered_map<int, Array<int>> vertex_edges_;

    // Faces' id for each edge.
    std::unordered_map<int, std::set<int>> edge_faces_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
