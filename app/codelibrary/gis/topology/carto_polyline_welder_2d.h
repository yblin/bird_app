//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_TOPOLOGY_CARTO_POLYLINE_WELDER_2D_H_
#define CODELIBRARY_GIS_TOPOLOGY_CARTO_POLYLINE_WELDER_2D_H_

#include <map>
#include <unordered_map>

#include "codelibrary/base/enumerate.h"
#include "codelibrary/geometry/polyline_util.h"
#include "codelibrary/geometry/transform_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/geometry/util/snap_2d.h"
#include "codelibrary/gis/carto_polyline_util.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace gis {

/**
 * Welder for overlapping 2D carto polyline kernels (resolve overlaps between
 * polylines to form a non-overlapping connecting network).
 */
class CartoPolylineWelder2D {
public:
    using RoadKernel = CartoPolylineKernel2D;

    /**
     * Constructor for CartoPolylineWelder2D.
     *
     * Parameters:
     *  overlap_distance_threshold - Max allowable min distance (m) for
     *                               overlapping judgment, default 0.5m.
     *  min_overlap_length         - Min valid overlap length (m) to trigger
     *                               welding, default 5.0m.
     */
    explicit CartoPolylineWelder2D(double overlap_distance_threshold = 0.5,
                                   double min_overlap_length = 5.0)
          : overlap_distance_threshold_(overlap_distance_threshold),
            min_overlap_length_(min_overlap_length) {
        CHECK(overlap_distance_threshold_ > 0.0);
        CHECK(min_overlap_length_ > 0.0);
        CHECK(min_overlap_length_ > 2.0 * overlap_distance_threshold_);
    }

    /**
     * Pre-weld only topologically consistent overlapping regions between two
     * road networks.
     *
     * Perform preliminary (intermediate) welding on two overlapping,
     * pre-consolidated road networks:
     *  1. Input constraint: Each network is internally consolidated (no
     *     overlapping/duplicate RoadKernel within a single network);
     *  2. Welding rule: Only pre-weld STABLE, topologically consistent
     *     overlapping regions (the only directly weldable parts);
     *  3. Skip criteria: Omit partial overlaps, topologically inconsistent
     *     regions, or non-overlapping areas (unstable for direct welding);
     *  4. Output characteristic: The two output road networks only complete
     *     pre-welding in consistent regions — unwelded parts remain, and
     *     full/further welding is required in subsequent processing steps.
     */
    void Preweld(const Array<RoadKernel>& road_network1,
                 const Array<RoadKernel>& road_network2,
                 Array<RoadKernel>* welded_road_network1,
                 Array<RoadKernel>* welded_road_network2) {
        CHECK(welded_road_network1);
        CHECK(welded_road_network2);

        if (road_network1.empty() || road_network2.empty()) {
            *welded_road_network1 = road_network1;
            *welded_road_network2 = road_network2;
            return;
        }

        Array<RBox2D> boxes;
        for (const auto& r : road_network1) {
            boxes.push_back(RBox2D(r.polyline.begin(), r.polyline.end()));
        }

        Array<bool> fixed1(road_network1.size(), false);
        Array<bool> fixed2(road_network2.size(), false);

        geometry::RTree2D<double> rtree;
        rtree.Construct(boxes);
        Array<int> entries;
        RoadKernel overlap;
        Array<RoadKernel> non_overlaps1, non_overlaps2;
        Array<RoadKernel> welded;
        Array<RoadKernel> unwelded1, unwelded2;

        for (auto&& [index, r2] : enumerate(road_network2)) {
            if (fixed2[index]) continue;

            RBox2D box(r2.polyline.begin(), r2.polyline.end());
            RBox2D expand_box(box.x_min() - overlap_distance_threshold_,
                              box.x_max() + overlap_distance_threshold_,
                              box.y_min() - overlap_distance_threshold_,
                              box.y_max() + overlap_distance_threshold_);
            rtree.Find(expand_box, &entries);
            for (int j : entries) {
                if (fixed1[j]) continue;
                if (road_network1[j].category != r2.category ||
                    road_network1[j].level != r2.level) continue;
                if (Weld(road_network1[j], r2, &overlap, &non_overlaps1,
                         &non_overlaps2)) {
                    unwelded1.insert(non_overlaps1);
                    unwelded2.insert(non_overlaps2);
                    welded.push_back(overlap);
                    fixed1[j] = true;
                    fixed2[index] = true;
                    break;
                }

                RoadKernel s = r2;
                s.polyline.Reverse();
                if (Weld(road_network1[j], s, &overlap, &non_overlaps1,
                         &non_overlaps2)) {
                    unwelded1.insert(non_overlaps1);
                    unwelded2.insert(non_overlaps2);
                    welded.push_back(overlap);
                    fixed1[j] = true;
                    fixed2[index] = true;
                    break;
                }
            }
        }
        for (auto&& [index, r] : enumerate(road_network1)) {
            if (!fixed1[index])
                unwelded1.push_back(r);
        }
        for (auto&& [index, r] : enumerate(road_network2)) {
            if (!fixed2[index])
                unwelded2.push_back(r);
        }
        PreweldNonOverlap(unwelded1, unwelded2, welded_road_network1,
                          welded_road_network2);
        welded_road_network1->insert(welded);
        welded_road_network2->insert(welded);

        // Use a tight width threshold here (not the spatial threshold) so that
        // overlap kernels welded with reconciled widths are not merged into
        // adjacent kernels of differing original widths — that merge would
        // re-average and drift across iterative preweld runs.
        const double merge_width_threshold = 1e-1;
        carto_polyline_util::MergeKernels(merge_width_threshold,
                                          welded_road_network1);
        carto_polyline_util::MergeKernels(merge_width_threshold,
                                          welded_road_network2);
    }

    /**
     * Pre-weld only topologically consistent overlapping regions between a
     * primary road network and several neighbor road networks.
     */
    void Preweld(const Array<RoadKernel>& road_network,
                 const Array<Array<RoadKernel>>& neighbor_road_networks,
                 Array<RoadKernel>* welded_road_network,
                 Array<Array<RoadKernel>>* welded_neighbors) {
        CHECK(welded_road_network);
        CHECK(welded_neighbors);
        CHECK(welded_neighbors != &neighbor_road_networks);

        welded_neighbors->clear();
        *welded_road_network = road_network;
        if (neighbor_road_networks.empty()) {
            return;
        }

        for (const auto& road_network2 : neighbor_road_networks) {
            Array<RoadKernel> welded_neighbor;
            Preweld(*welded_road_network, road_network2, welded_road_network,
                    &welded_neighbor);
            welded_neighbors->push_back(welded_neighbor);
        }
    }

private:
    /**
     * Get connected components of road segments based on polyline spatial
     * connectivity.
     */
    static void GetConnectedComponents(const Array<RoadKernel>& road_segments,
                                       Array<Array<int>>* components) {
        CHECK(components);

        components->clear();
        DisjointSet set(road_segments.size());

        std::map<RPoint2D, Array<int>> graph;
        for (auto&& [index, r] : enumerate(road_segments)) {
            graph[r.polyline.front()].push_back(index);
            graph[r.polyline.back()].push_back(index);
        }
        for (auto p : graph) {
            for (auto& j : p.second) {
                set.Union(p.second[0], j);
            }
        }
        set.ToClusters(components);
    }

    /**
     * Find the longest overlapping parts between two road kernels, then "weld"
     * and split them into remaining parts.
     *
     * "Weld" means aligning the two road kernels at the overlapping part, then
     * splitting each kernel into non-overlapping remaining parts.
     */
    bool Weld(const RoadKernel& branch1,
              const RoadKernel& branch2,
              RoadKernel* overlap_part,
              Array<RoadKernel>* remain_branches1,
              Array<RoadKernel>* remain_branches2) const {
        CHECK(overlap_part);
        CHECK(remain_branches1);
        CHECK(remain_branches2);
        CHECK(branch1.category == branch2.category);
        CHECK(branch1.level == branch2.level);

        overlap_part->clear();
        remain_branches1->clear();
        remain_branches2->clear();

        if (branch1.empty() || branch2.empty()) {
            return false;
        }

        if (std::equal(branch1.polyline.begin(), branch1.polyline.end(),
                       branch2.polyline.begin(), branch2.polyline.end())) {
            *overlap_part = branch1;
            overlap_part->width = std::max(branch1.width, branch2.width);
            return true;
        }

        double l1 = branch1.polyline.Length(), l2 = branch2.polyline.Length();
        if (l1 < min_overlap_length_ || l2 < min_overlap_length_) {
            return false;
        }

        const double step = 0.5 * overlap_distance_threshold_;
        CHECK(step * INT_MAX > l1);
        CHECK(step * INT_MAX > l2);

        RPolyline2D polyline1, polyline2;
        geometry::polyline_util::Densify(branch1.polyline, step, &polyline1);
        geometry::polyline_util::Densify(branch2.polyline, step, &polyline2);
        Array<double> overlap(polyline2.size(), DBL_MAX);
        geometry::Snap2D<double> snap(polyline1, overlap_distance_threshold_);
        for (int i = 0; i < polyline2.size(); ++i) {
            RPoint2D q;
            if (snap.FindSnapVertex(polyline2[i], &q)) {
                overlap[i] = Distance(q, polyline2[i]);
            }
        }

        // Find longest overlap interval.
        std::pair<int, int> longest = {-1, -1};
        int start = -1;
        double max_len = 0.0;

        Array<double> lengths(overlap.size());
        lengths[0] = 0.0;
        for (int i = 1; i < overlap.size(); ++i) {
            lengths[i] = lengths[i - 1] + Distance(polyline2[i],
                                                   polyline2[i - 1]);
        }
        for (int i = 0; i < overlap.size(); ++i) {
            if (overlap[i] < overlap_distance_threshold_) {
                if (start == -1) start = i;
                double l = lengths[i] - lengths[start];
                if (l > max_len) {
                    max_len = l;
                    longest = {start, i};
                }
            } else {
                start = -1;
            }
        }
        ++longest.first;
        --longest.second;

        if (max_len > min_overlap_length_ && longest.second > longest.first) {
            int v1 = geometry::polyline_util::FindNearestVertex(
                        polyline1, polyline2[longest.first]);
            int v2 = geometry::polyline_util::FindNearestVertex(
                        polyline1, polyline2[longest.second]);
            if (v1 != -1) v1 = std::max(v1, 1);
            if (v2 != -1) v2 = std::min(v2, polyline1.size() - 2);
            if (v1 == -1 || v2 == -1 || v1 >= v2) {
                return false;
            }

            // Find the best place to weld two segments.
            int best = -1;
            double dis = DBL_MAX;
            for (int j = longest.first; j < longest.second; ++j) {
                if (overlap[j] + overlap[j + 1] < dis) {
                    dis = overlap[j] + overlap[j + 1];
                    best = j;
                }
            }
            CHECK(best != -1);
            RPoint2D p1 = geometry::Project(polyline2[best], branch1.polyline);
            RPoint2D p2 = geometry::Project(polyline2[best + 1],
                                            branch1.polyline);
            RPolyline2D poly1, poly2, poly3, poly4;
            geometry::polyline_util::Split(branch1.polyline, p1,
                                           &poly1, &poly4);
            if (poly4.size() <= 1) return false;
            geometry::polyline_util::Split(poly4, p2, &poly2, &poly3);

            *overlap_part = branch1;
            overlap_part->polyline = poly2;
            overlap_part->width = std::max(branch1.width, branch2.width);

            if (poly1.size() >= 2) {
                RoadKernel s1 = branch1;
                s1.polyline = poly1;
                remain_branches1->push_back(s1);
            }
            if (poly3.size() >= 2) {
                RoadKernel s2 = branch1;
                s2.polyline = poly3;
                remain_branches1->push_back(s2);
            }

            geometry::polyline_util::Split(branch2.polyline, p1, &poly1,
                                           &poly4);
            if (poly4.size() <= 1) return false;
            geometry::polyline_util::Split(poly4, p2, &poly2, &poly3);
            if (poly1.size() >= 2) {
                RoadKernel s1 = branch2;
                s1.polyline = poly1;
                remain_branches2->push_back(s1);
            }
            if (poly3.size() >= 2) {
                RoadKernel s2 = branch2;
                s2.polyline = poly3;
                remain_branches2->push_back(s2);
            }

            return true;
        }

        return false;
    }

    /**
     * Weld non-overlap parts of road network.
     */
    void PreweldNonOverlap(const Array<RoadKernel>& non_welded1,
                           const Array<RoadKernel>& non_welded2,
                           Array<RoadKernel>* welded_road_network1,
                           Array<RoadKernel>* welded_road_network2) const {
        *welded_road_network1 = non_welded1;
        welded_road_network2->clear();

        // Get independent subgraph.
        Array<Array<int>> clusters1, clusters2;
        GetConnectedComponents(non_welded1, &clusters1);
        GetConnectedComponents(non_welded2, &clusters2);

        std::map<RPoint2D, int> degrees1, degrees2;
        for (auto& r : non_welded1) {
            ++degrees1[r.polyline.front()];
            ++degrees1[r.polyline.back()];
        }
        for (auto& r : non_welded2) {
            ++degrees2[r.polyline.front()];
            ++degrees2[r.polyline.back()];
        }

        // Using 1-degree endpoint point as a key to map subgraph node.
        using Feature = std::set<RPoint2D>;

        std::map<Feature, Array<Array<int>>> graph_hash1, graph_hash2;
        for (auto& c : clusters1) {
            std::set<RPoint2D> s;
            for (auto j : c) {
                const RPolyline2D& polyline = non_welded1[j].polyline;
                if (degrees1[polyline.front()] == 1) s.insert(polyline.front());
                if (degrees1[polyline.back()] == 1) s.insert(polyline.back());
            }
            graph_hash1[s].push_back(c);
        }
        for (auto& c : clusters2) {
            std::set<RPoint2D> s;
            for (auto j : c) {
                const RPolyline2D& polyline = non_welded2[j].polyline;
                if (degrees2[polyline.front()] == 1) s.insert(polyline.front());
                if (degrees2[polyline.back()] == 1) s.insert(polyline.back());
            }
            graph_hash2[s].push_back(c);
        }

        for (auto& p : graph_hash2) {
            if (p.first.empty() ||
                p.second.size() != 1 || graph_hash1[p.first].size() != 1) {
                for (auto& c : p.second) {
                    for (int j : c) {
                        welded_road_network2->push_back(non_welded2[j]);
                    }
                }

                continue;
            }

            for (auto& c : graph_hash1[p.first]) {
                for (int j : c) {
                    welded_road_network2->push_back(non_welded1[j]);
                }
            }
        }
    }

    // Max allowable min distance for overlapping judgment.
    double overlap_distance_threshold_;

    // Min valid overlap length (m) to trigger welding.
    double min_overlap_length_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_TOPOLOGY_CARTO_POLYLINE_WELDER_2D_H_
