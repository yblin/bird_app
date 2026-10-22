//
// Copyright 2021-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_PLANAR_ARRANGEMENT_3D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_PLANAR_ARRANGEMENT_3D_H_

#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/util/pad_3d.h"

namespace cl {
namespace geometry {

/**
 * PlanarArrangement3D similars to Arragment2D. The only difference is that
 * PlanarArrangement3D parition the given 3D plane, while Arrangment2D only lies
 * on XY plane.
 *
 * More details see Arrangement2D.
 */
template <typename T>
class PlanarArrangement3D : public Pad3D<T> {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using Point3   = Point3D<T>;
    using Point2   = Point2D<T>;
    using Segment3 = Segment3D<T>;
    using Segment2 = Segment2D<T>;
    using Vertex   = typename Arrangement2D<T>::Vertex;

    /**
     * Construct an empty arrangement.
     *
     * Users can use Reset() to initialize the arrangement latter.
     */
    explicit PlanarArrangement3D(double threshold = 0.0)
        : arrangement2d_(threshold) {}

    explicit PlanarArrangement3D(const Plane3D<T>& plane,
                                 double threshold = 0.0)
        : arrangement2d_(threshold) {
        Reset(plane);
    }

    PlanarArrangement3D(const PlanarArrangement3D&) = delete;

    PlanarArrangement3D& operator= (const PlanarArrangement3D&) = delete;

    void Reset(const Plane3D<T>& plane) {
        this->clear();
        this->ResetPlane(plane);
    }

    /**
     * Insert a set of 3D line segments.
     */
    void Insert(const Array<Segment3>& segs) {
        for (const Segment3& s : segs) {
            Insert(s);
        }
    }

    /**
     * Insert a 3D line segment.
     *
     * The time complexity is O(N * logN).
     */
    void Insert(const Segment3& seg) {
        arrangement2d_.Insert(this->Project(seg.lower_point()),
                              this->Project(seg.upper_point()));
    }
    void Insert(const Point3& p1, const Point3& p2, int color1 = 0,
                int color2 = 0) {
        arrangement2d_.Insert(this->Project(p1), this->Project(p2),
                              color1, color2);
    }
    void Insert2D(const Point2& p1, const Point2& p2, int color1 = 0,
                  int color2 = 0) {
        arrangement2d_.Insert(p1, p2, color1, color2);
    }
    void InsertWithoutCross(const Point3& p1, const Point3& p2,
                            int color1 = 0, int color2 = 0) {
        arrangement2d_.InsertWithoutCross(this->Project(p1),
                                          this->Project(p2),
                                          color1,
                                          color2);
    }
    void InsertWithoutNewVertex(const Point3& p1, const Point3& p2,
                                int color1 = 0, int color2 = 0) {
        arrangement2d_.InsertWithoutNewVertex(this->Project(p1),
                                              this->Project(p2),
                                              color1,
                                              color2);
    }

    /**
     * Insert a line with snaping.
     */
    void SnapInsert(const Point3& s, const Point3& t,
                    int color1 = 0, int color2 = 0) {
        arrangement2d_.SnapInsert(this->Project(s), this->Project(t), color1,
                                  color2);
    }
    void SnapInsert2D(const Point2& s, const Point2& t,
                      int color1 = 0, int color2 = 0) {
        arrangement2d_.SnapInsert(s, t, color1, color2);
    }

    /**
     * Insert an incident 3D point and return the inserted vertex.
     */
    Vertex* Insert(const Point3& p) {
        return arrangement2d_.Insert(this->Project(p));
    }

    /**
     * Insert an incident 2D point and split the incident edges.
     */
    Vertex* SnapInsert2D(const Point2& p) {
        return arrangement2d_.SnapInsert(p);
    }

    /**
     * Return the nearest vertex of 'p'.
     */
    Vertex* FindNearestVertex(const Point3& p) {
        return arrangement2d_.FindNearestVertex(this->Project(p));
    }

    /**
     * Get snap edges from s to t.
     */
    void GetSnapEdge(const Point3& s, const Point3& t, Array<Point3>* path) {
        CHECK(path);

        Array<Point2> path2d;
        arrangement2d_.GetSnapEdge(this->Project(s), this->Project(t),
                                   &path2d);

        path->clear();
        for (auto p : path2d) {
            path->push_back(this->Unproject(p));
        }
    }

    /**
     * Get partition regions enclosed by line segments.
     *
     * Each region consist of serval connected inner triangles. Each triangles
     * is defined by one of its halfedge.
     */
    void Arrange() {
        arrangement2d_.Arrange();
    }

    /**
     * Get the region that contains the given 3D point.
     *
     * Make sure 'Arrange()' is called before.
     *
     * It returns -1 if no region is found.
     */
    int Locate(const Point3& p) const {
        return arrangement2d_.Locate(Project(this->plane_, p));
    }

    void clear() {
        arrangement2d_.clear();
    }

    bool empty() const {
        return arrangement2d_.empty();
    }

    Arrangement2D<T>& arrangement2d() {
        return arrangement2d_;
    }

    const Arrangement2D<T>& arrangement2d() const {
        return arrangement2d_;
    }

    double threshold() const {
        return arrangement2d_.threshold();
    }

    void set_color_policy(typename Delaunay2D<T>::ColorPolicy policy) {
        arrangement2d_.set_color_policy(policy);
    }

protected:
    // 2D arrangement.
    Arrangement2D<T> arrangement2d_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_PLANAR_ARRANGEMENT_3D_H_
