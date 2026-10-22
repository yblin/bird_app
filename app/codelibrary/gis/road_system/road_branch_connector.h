//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_CONNECTOR_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_CONNECTOR_H_

#include <map>
#include <unordered_set>

#include "codelibrary/base/arange.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/geometry/curve/cubic_hermite_spline_2d.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/generator_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/range_ray_2d.h"
#include "codelibrary/gis/util/grid_rtree_2d.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace gis {

/**
 * RoadBranchConnector responsible for connecting road branches (polylines)
 * by finding optimal junctions and generating smooth connection.
 */
class RoadBranchConnector {
public:
    /**
     * Road branch structure with dual junctions (head and tail), containing the
     * main polyline and junction vertex indices.
     */
    struct BranchWithJunctions {
        RPolyline2D polyline;

        // Each road branch has exactly two junctions, corresponding to its two
        // endpoints.
        int junction_vertex1, junction_vertex2;

        const RPoint2D& junction_point1() const {
            CHECK(junction_vertex1 >= 0);
            CHECK(junction_vertex1 < polyline.size());

            return polyline[junction_vertex1];
        }

        const RPoint2D& junction_point2() const {
            CHECK(junction_vertex2 >= 0);
            CHECK(junction_vertex2 < polyline.size());

            return polyline[junction_vertex2];
        }

        /**
         * The segment of the polyline that lies between the two junction
         * vertices, excluding junction-related part.
         */
        void GetNonJunctionPolyline(RPolyline2D* non_junction_polyline) const {
            CHECK(non_junction_polyline);

            if (polyline.size() <= 1) {
                non_junction_polyline->clear();
                return;
            }

            CHECK(junction_vertex1 <= junction_vertex2);
            *non_junction_polyline =
                    RPolyline2D(polyline.begin() + junction_vertex1,
                                polyline.begin() + junction_vertex2 + 1);
        }
    };

    /**
     * Connection between a single junction and another connected entity (which
     * can be a line, pair-connection, or other objects).
     */
    struct SingleConnection {
        // Index of branch junction.
        int junction;

        // Vertex index on the corresponding branch polyline.
        int vertex;

        // Width of the junction.
        double width;

        // Connection score.
        double score = DBL_MAX;

        // Line segment that connects the junction to the target entity.
        RSegment2D line;

        // Specific point on the target entity where the connection line
        // terminates.
        RPoint2D connection_point;

        // Specific junction point to connect to the target.
        RPoint2D junction_point;

        /**
         * Compare single connections by their scores.
         * The smaller the score, the better the connection.
         */
        bool operator< (const SingleConnection& rhs) const {
            return score < rhs.score;
        }
    };

    /**
     * Connection for two branches.
     */
    struct PairConnection {
        // Indices of branch junctions.
        int junction1, junction2;

        // Vertex indices on the corresponding branch polylines.
        int vertex1, vertex2;

        // Width of junction1 and junction2.
        double width1, width2;

        // Connection score.
        double score = DBL_MAX;

        // Line that connects two junctions.
        RSegment2D line;

        // Connected junction points (equals to line).
        RPoint2D junction_point1, junction_point2;

        /**
         * Compare pair connections by their scores.
         * The smaller the score, the better the connection.
         */
        bool operator< (const PairConnection& rhs) const {
            return score < rhs.score;
        }
    };

    explicit RoadBranchConnector(double proximity_factor = 1.0)
        : proximity_factor_(proximity_factor) {
        CHECK(proximity_factor_ > 0.0);
    }

    RoadBranchConnector(const RoadBranchConnector&) = delete;

    RoadBranchConnector& operator=(const RoadBranchConnector&) = delete;

    bool empty() const {
        return branches_.empty();
    }

    void clear() {
        branches_.clear();
        accumulated_length_.clear();
        tangent_vectors_.clear();
    }

    /**
     * Reset road boundaries.
     */
    void ResetRoadPolygon(const RMultiPolygon2D& road_polygon) {
        road_polygon_ = road_polygon;
        road_boundaries_.clear();
        for (const auto& b : road_polygon.boundaries()) {
            for (int i = 0; i < b.polygon.size(); ++i) {
                road_boundaries_.push_back(b.polygon.edge(i));
            }
        }
        rtree_.Construct(road_boundaries_);
    }

    /**
     * Reset input road branches.
     */
    void ResetBranches(const Array<RPolyline2D>& branches) {
        branches_.resize(branches.size());
        accumulated_length_.resize(2 * branches.size());
        tangent_vectors_.resize(2 * branches.size());
        for (int i = 0; i < branches_.size(); ++i) {
            UpdateBranch(i, branches[i]);
        }
    }

    /**
     * Find the optimal pair-connection between two junctions.
     */
    bool FindPairConnection(int junction1, int junction2,
                            double width1, double width2,
                            PairConnection* connection) const {
        CHECK(junction1 >= 0 && junction1 / 2 < branches_.size());
        CHECK(junction2 >= 0 && junction2 / 2 < branches_.size());
        CHECK(junction1 != junction2);
        CHECK(connection);

        if (width1 <= 0.0 || width2 <= 0.0) return false;

        const BranchWithJunctions& branch1 = branches_[junction1 / 2];
        const BranchWithJunctions& branch2 = branches_[junction2 / 2];

        if (branch1.polyline.size() < 2 ||
            branch2.polyline.size() < 2) return false;

        auto arange1 = junction1 % 2 == 0 ? Arange(branch1.junction_vertex2 + 1)
                                          : Arange(branch1.polyline.size() - 1,
                                                   branch1.junction_vertex1 - 1,
                                                   -1);
        auto arange2 = junction2 % 2 == 0 ? Arange(branch2.junction_vertex2 + 1)
                                          : Arange(branch2.polyline.size() - 1,
                                                   branch2.junction_vertex1 - 1,
                                                   -1);

        connection->score = DBL_MAX;
        PairConnection c;
        c.junction1 = junction1;
        c.junction2 = junction2;
        c.width1 = width1;
        c.width2 = width2;
        for (int i : arange1) {
            double l1 = accumulated_length_[junction1][i];
            if (l1 > width1) break;
            const RPoint2D& p1 = branch1.polyline[i];
            const RVector2D& v1 = tangent_vectors_[junction1][i];
            for (int j : arange2) {
                double l2 = accumulated_length_[junction2][j];
                if (l2 > width2) break;
                if (junction1 / 2 == junction2 / 2) {
                    // Two junctions are on the same branch.
                    if (i == j) break;
                }

                const RPoint2D& p2 = branch2.polyline[j];
                const RVector2D& v2 = tangent_vectors_[junction2][j];
                c.vertex1 = i;
                c.vertex2 = j;
                c.junction_point1 = p1;
                c.junction_point2 = p2;
                c.line = RSegment2D(p1, p2);
                double d1 = JunctionLineDistance(junction1, i, c.line);
                double d2 = JunctionLineDistance(junction2, j, c.line);
                c.score = Degree(p2 - p1, v1) + Degree(p1 - p2, v2);
                double area1 = 2.0 * d1 * l1, area2 = 2.0 * d2 * l2;
                if (area1 / width1 > proximity_factor_ ||
                    area2 / width2 > proximity_factor_) continue;
                if (c < *connection) {
                    *connection = c;
                }
            }
        }

        return connection->score < DBL_MAX;
    }

    /**
     * Find the optimal connection between a junction and a line segment.
     */
    bool FindSingleConnection(int junction, double width,
                              const RSegment2D& line,
                              SingleConnection* connection) const {
        CHECK(junction >= 0 && junction / 2 < branches_.size());
        CHECK(connection);

        if (line.lower_point() == line.upper_point()) return false;

        const BranchWithJunctions& branch = branches_[junction / 2];
        if (branch.polyline.size() < 2) return false;

        auto arange = junction % 2 == 0 ? Arange(branch.junction_vertex2 + 1)
                                        : Arange(branch.polyline.size() - 1,
                                                 branch.junction_vertex1 - 1,
                                                 -1);

        connection->score = DBL_MAX;
        SingleConnection c;
        c.junction = junction;
        c.width = width;

        Array<RPoint2D> points;
        for (int i : arange) {
            double l = accumulated_length_[junction][i];
            if (l > width) {
                double remain = l - width;
                if (!points.empty()) {
                    RVector2D v = points.back() - branch.polyline[i];
                    RPoint2D q = branch.polyline[i] + v.normalized() * remain;
                    points.push_back(q);
                }
                break;
            }
            points.push_back(branch.polyline[i]);
        }

        for (int i : arange) {
            double l = accumulated_length_[junction][i];
            if (l > width) break;
            const RPoint2D& p = branch.polyline[i];
            const RVector2D& v = tangent_vectors_[junction][i];

            RRangeRay2D ray(p, v);
            RPoint2D q;
            if (!ray.Hit(line, &q)) {
                double d1 = Degree(v, line.lower_point() - p);
                double d2 = Degree(v, line.upper_point() - p);
                if (d1 < d2)
                    q = line.lower_point();
                else
                    q = line.upper_point();
            }

            c.vertex = i;
            c.line = RSegment2D(p, q);
            c.connection_point = q;
            c.junction_point = p;
            const double d = JunctionLineDistance(junction, i, c.line);
            const double area = 2.0 * d * l;
            if (area / width > proximity_factor_) continue;
            if (!IsValid(c.line)) continue;

            RSegment2D line(q, points.back());
            c.score = 0.0;
            for (auto q : points) {
                c.score += Distance(q, line);
            }

            if (c < *connection) {
                *connection = c;
            }
        }

        return connection->score < DBL_MAX;
    }

    /**
     * Update the connection point of the given single-connection.
     *
     * Return false if the new connection is invalid.
     */
    bool UpdateConnectionPoint(const RPoint2D& p,
                               SingleConnection* connection) const {
        CHECK(connection);

        double l = accumulated_length_[connection->junction]
                                      [connection->vertex];
        connection->connection_point = p;
        connection->line = RSegment2D(connection->junction_point, p);

        const double d = JunctionLineDistance(connection->junction,
                                              connection->vertex,
                                              connection->line);
        const double area = 2.0 * d * l;
        if (area / connection->width > proximity_factor_) return false;

        return this->IsValid(connection->line);
    }

    /**
     * Iteratively connect given junctions using pair connections.
     *
     * The core of this function is to determine whether or not two junctions
     * belong to the same road.
     */
    bool PairConnect(const Array<int>& junctions, const Array<double>& width,
                     double score_threshold,
                     Array<PairConnection>* connections) {
        CHECK(connections);
        CHECK(junctions.size() >= 2);
        CHECK(junctions.size() == width.size());
        CHECK(score_threshold > 0.0);

        connections->clear();
        const int n = junctions.size();
        if (n <= 1) return false;

        // Iteratively find and establish the best pair connections.
        std::map<int, bool> finish;
        while (true) {
            PairConnection connection;
            Array<PairConnection> candidates;
            for (int i = 0; i < n; ++i) {
                int j1 = junctions[i];
                if (finish[j1]) continue;
                for (int j = i + 1; j < n; ++j) {
                    int j2 = junctions[j];
                    if (finish[j2]) continue;

                    double w1 = width[i];
                    double w2 = width[j];
                    if (FindPairConnection(j1, j2, w1, w2, &connection)) {
                        candidates.push_back(connection);
                    }
                }
            }

            if (candidates.empty()) break;
            std::sort(candidates.begin(), candidates.end());

            if (candidates.front().score > score_threshold) break;

            // For a bidirectional road, two junctions often correspond to its
            // left and right lanes. When a third junction needs to connect to
            // this road, it should ideally link to the intersection point of
            // these two lane junctions rather than either lane individually.
            // This logic identifies such intermediate junctions by analyzing
            // connection scores.
            if (candidates.size() >= 2) {
                if (candidates[1].score < 2.0 * candidates[0].score &&
                    (candidates[0].junction1 == candidates[1].junction1 ||
                     candidates[0].junction1 == candidates[1].junction2 ||
                     candidates[0].junction2 == candidates[1].junction1 ||
                     candidates[0].junction2 == candidates[1].junction2))
                    break;
            }

            bool found = false;
            for (PairConnection& best : candidates) {
                if (best.score > score_threshold) break;

                double r = 0.1 * (best.width1 + best.width2);
                if (!IsValid(best.line, r)) return false;

                // Check if the connection is crossed with the branch.
                bool crossed = false;
                for (auto j : junctions) {
                    auto& branch = branches_[j / 2];
                    if (j / 2 == best.junction1 / 2 ||
                        j / 2 == best.junction2 / 2)
                        continue;
                    for (int i = 0; i + 1 < branch.polyline.size(); ++i) {
                        RSegment2D s(branch.polyline[i],
                                     branch.polyline[i + 1]);
                        if (IsCross(best.line, s, r)) {
                            crossed = true;
                            break;
                        }
                    }
                    if (crossed) break;
                }
                if (crossed) continue;

                finish[best.junction1] = true;
                finish[best.junction2] = true;

                connections->push_back(best);
                SetJunctionVertex(best.junction1, best.vertex1);
                SetJunctionVertex(best.junction2, best.vertex2);
                found = true;
                break;
            }
            if (!found) return false;
        }
        return !connections->empty();
    }

    /**
     * Connect the given ordered junctions to line segment.
     *
     * The input junctions must be sorted relative to line to maintain the
     * non-crossing property of connections.
     */
    bool ConnectJunctionsToLine(const Array<int>& junctions,
                                const Array<double>& width,
                                const RSegment2D& line,
                                Array<SingleConnection>* connections) {
        CHECK(connections);
        CHECK(junctions.size() == width.size());

        connections->clear();

        Array<SingleConnection> single_connections;

        // Check if the input junctions contains duplicate elements.
        // Duplicate junctions may indicate the presence of a cycle in the road
        // network structure. Such cycles are often caused by holes.
        // For these cases, we directly return false and skip further processing
        // to avoid incorrect results.
        std::unordered_set<int> seen;
        for (int j : junctions) {
            if (seen.count(j)) return false;
            seen.insert(j);
        }

        Array<double> scores;
        for (int i = 0; i < junctions.size(); ++i) {
            int j = junctions[i];

            SingleConnection connection;
            if (!FindSingleConnection(j, width[i], line, &connection)) {
                // Remeber to restore the junction vertices.
                for (int k = 0; k < i; ++k) {
                    UnsetJunctionVertex(junctions[k]);
                }
                return false;
            }

            scores.push_back(connection.score);
            SetJunctionVertex(j, connection.vertex);
            single_connections.push_back(connection);
        }

        CHECK(single_connections.size() == junctions.size());

        Array<int> indices;
        IndexSort(scores.begin(), scores.end(), &indices);

        Array<RSegment2D> connection_lines;
        Array<RPoint2D> candidates;
        for (int i : indices) {
            auto& c = single_connections[i];
            RSegment2D l1 = c.line;
            bool l1_invalid = false;
            for (auto l : connection_lines) {
                if (geometry::Cross(l1, l)) {
                    l1_invalid = true;
                    break;
                }
            }
            if (l1_invalid) {
                double best = DBL_MAX;
                for (auto p : candidates) {
                    RSegment2D l2(c.junction_point, p);
                    bool l2_invalid = false;
                    for (auto l3 : connection_lines) {
                        if (geometry::Cross(l2, l3)) {
                            l2_invalid = true;
                            break;
                        }
                    }
                    if (l2_invalid) continue;

                    RVector2D v = tangent_vectors_[c.junction][c.vertex];
                    double s = Degree(v, p - c.junction_point);
                    if (s < best && UpdateConnectionPoint(p, &c)) {
                        best = s;
                    }
                }
                if (best == DBL_MAX) {
                    // Remeber to restore the junction vertices.
                    for (int j : junctions) {
                        UnsetJunctionVertex(j);
                    }

                    return false;
                }
                connection_lines.push_back(c.line);
            } else {
                connection_lines.push_back(l1);
                candidates.push_back(c.connection_point);
            }
        }

        *connections = single_connections;
        return true;
    }

    /**
     * Connect the given branch to road polygon's boundary.
     */
    bool ConnectBranchToBoundary(const RPoint2D& p, const RVector2D& v,
                                 double extend_length,
                                 RPoint2D* connection) const {
        CHECK(extend_length > 0.0);
        CHECK(connection);

        RSegment2D s(p, p + v.normalized() * extend_length);
        Array<int> entries;
        rtree_.Find(s.bounding_box(), &entries);
        double dis = DBL_MAX;
        for (int i : entries) {
            Array<RPoint2D> ps;
            if (geometry::Intersect(road_boundaries_[i], s, &ps)) {
                for (auto q : ps) {
                    double d = Distance(q, p);
                    if (d < dis) {
                        dis = d;
                        *connection = q;
                    }
                }
            }
        }
        return dis < DBL_MAX;
    }

    /**
     * Call if the i-th branch is updated.
     */
    void UpdateBranch(int i, const RPolyline2D& branch) {
        CHECK(i >= 0 && i < branches_.size());

        branches_[i].polyline = branch;
        branches_[i].junction_vertex1 = 0;
        branches_[i].junction_vertex2 = branch.size() - 1;
        auto& len1 = accumulated_length_[2 * i];
        auto& vec1 = tangent_vectors_[2 * i];
        int n = branch.size();

        if (n == 0) {
            len1.clear();
            vec1.clear();
        } else {
            len1.resize(n);
            vec1.resize(n);
            len1[0] = 0.0;
            for (int j = 1; j < n; ++j) {
                len1[j] = Distance(branch[j], branch[j - 1]) + len1[j - 1];
                vec1[j - 1] = (branch[j - 1] - branch[j]).normalized();
            }
            if (n == 1) {
                vec1[n - 1] = RVector2D(0.0, 0.0);
            } else {
                vec1[n - 1] = vec1[n - 2];
            }
        }

        auto& len2 = accumulated_length_[2 * i + 1];
        auto& vec2 = tangent_vectors_[2 * i + 1];
        if (n == 0) {
            len2.clear();
            vec2.clear();
        } else {
            len2.resize(n);
            vec2.resize(n);
            len2[n - 1] = 0.0;
            for (int j = n - 2; j >= 0; --j) {
                len2[j] = Distance(branch[j], branch[j + 1]) + len2[j + 1];
                vec2[j + 1] = (branch[j + 1] - branch[j]).normalized();
            }
            if (n == 1) {
                vec2[0] = RVector2D(0.0, 0.0);
            } else {
                vec2[0] = vec2[1];
            }
        }
    }

    /**
     * Check if the line segment is a valid connection: all road boundaries are
     * within the specified threshold distance 'buffer' from this line segment.
     */
    bool IsValid(const RSegment2D& line, double buffer = 0.0) const {
        CHECK(buffer >= 0.0);

        Array<int> entries;
        rtree_.Find(line.bounding_box(), &entries);
        for (int i : entries) {
            if (IsCross(line, road_boundaries_[i], buffer)) return false;
        }
        return true;
    }

    const Array<BranchWithJunctions>& branches() const {
        return branches_;
    }

private:
    /**
     * Check if two line segments are crossed with buffer.
     */
    static bool IsCross(const RSegment2D& s1, const RSegment2D& s2,
                        double buffer) {
        if (geometry::Intersect(s1, s2)) return true;
        if (buffer == 0.0) return false;

        if (Distance(s1.lower_point(), s2) < buffer ||
            Distance(s1.upper_point(), s2) < buffer) {
            return true;
        }
        if (Distance(s2.lower_point(), s1) < buffer ||
            Distance(s2.upper_point(), s1) < buffer) {
            return true;
        }

        return false;
    }

    /**
     * Compute mean distance from junction to the connection line.
     */
    double JunctionLineDistance(int junction, int vertex,
                                const RSegment2D& connection_line) const {
        CHECK(junction >= 0);
        CHECK(junction / 2 < branches_.size());

        const BranchWithJunctions& branch = branches_[junction / 2];

        double dis = 0.0;
        if (junction % 2 == 0) {
            for (int k = 0; k < vertex; ++k) {
                dis += Distance(branch.polyline[k], connection_line);
            }
            if (vertex != 0) dis /= vertex;
        } else {
            for (int k = vertex + 1; k < branch.polyline.size(); ++k) {
                dis += Distance(branch.polyline[k], connection_line);
            }
            if (vertex + 1 != branch.polyline.size())
                dis /= branch.polyline.size() - vertex - 1;
        }
        return dis;
    }

    /**
     * Set junction vertex.
     */
    void SetJunctionVertex(int junction, int vertex) {
        CHECK(junction >= 0);
        CHECK(junction / 2 < branches_.size());

        BranchWithJunctions& branch = branches_[junction / 2];
        CHECK(vertex >= 0);
        CHECK(vertex < branch.polyline.size());

        if (junction % 2 == 0) {
            branch.junction_vertex1 = vertex;
        } else {
            branch.junction_vertex2 = vertex;
        }
    }

    /**
     * Restore the junction vertex.
     */
    void UnsetJunctionVertex(int junction) {
        CHECK(junction >= 0);
        CHECK(junction / 2 < branches_.size());

        BranchWithJunctions& branch = branches_[junction / 2];
        if (junction % 2 == 0) {
            branch.junction_vertex1 = 0;
        } else {
            branch.junction_vertex2 = branch.polyline.size() - 1;
        }
    }

    // Factor controlling how closely the generated connection line adheres to
    // the original branch.
    //
    // Typically, this value is set close to the resolution of the centerline
    // (i.e., the approximate distance between consecutive vertices in the
    // centerline polyline).
    // See RoadCenterline for more details.
    double proximity_factor_;

    // Pre-computed accumulated length for junctions.
    Array<Array<double>> accumulated_length_;

    // Tangent vector for each branch point.
    Array<Array<RVector2D>> tangent_vectors_;

    // The input all branches.
    Array<BranchWithJunctions> branches_;

    // Input road polygon.
    RMultiPolygon2D road_polygon_;

    // Road boundaries (edges of road_polygon_).
    Array<RSegment2D> road_boundaries_;

    // RTree for cross test.
    geometry::RTree2D<double> rtree_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_BRANCH_CONNECTOR_H_
