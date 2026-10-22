//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_EXTRACTOR_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_EXTRACTOR_H_

#include <map>
#include <utility>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/topology/even_odd_polygon_2d.h"
#include "codelibrary/geometry/topology/geo_graph_util.h"
#include "codelibrary/gis/carto_polyline.h"
#include "codelibrary/gis/carto_polyline_util.h"
#include "codelibrary/gis/road_system/road_categorizer.h"
#include "codelibrary/gis/road_system/road_centerline_optimizer.h"
#include "codelibrary/gis/topology/land_region_arrangement.h"

namespace cl {
namespace gis {

/**
 * Road centerline extractor.
 */
class RoadCenterlineExtractor {
public:
    using Road = CartoPolyline2D;
    using RoadKernel = CartoPolylineKernel2D;
    using Branch = RoadCenterlineOptimizer::Branch;

    /**
     * Core parameter configuration for road centerline extraction.
     */
    struct Parameter {
        // The spatial resolution (in linear units, e.g., meters) controlling
        // the densification of road polygon boundaries.
        //
        // This critical parameter defines the maximum distance between
        // consecutive vertices when discretizing the input road polygon edges.
        // A smaller value results in more densely spaced vertices along the
        // boundaries, capturing finer geometric details but increasing
        // computational load.
        //
        // It directly influences the density of vertices in the resulting
        // medial axis: denser boundary vertices (from smaller resolution) lead
        // to a more detailed medial axis with more vertices.
        //
        // Typical values depend on road scale and required detail (e.g.,
        // 2.0 meters for urban road networks, and 0.2 meters for narrow
        // sidewalks and crosswalks).
        double resolution = 1.0;
        // Minimum cut distance for pruning raw road centerlines.
        double min_cut_distance = 2.0;
        // Maximum allowable tolerance for road geometry simplification.
        double max_tolerance = 2.0;
    };

    /**
     * Constructor for RoadCenterline.
     */
    explicit RoadCenterlineExtractor(const Parameter& param)
        : param_(param) {
        CHECK(param_.resolution > 0.0);
        CHECK(param_.max_tolerance > 0.0);
        CHECK(param_.min_cut_distance > 0.0);
    }

    /**
     * Constructor for RoadCenterline.
     *
     * Parameters:
     *  aoi_polygon - AOI polygon for centerline filtering. Only centerlines
     *                falling within this polygon are saved.
     *  resolution - Spatial resolution (in meters), which controls the
     *               densification of road polygon boundaries and influences the
     *               detail level of the resulting medial axis.
     */
    explicit RoadCenterlineExtractor(const RMultiPolygon2D& aoi_polygon,
                                     const Parameter& param)
        : param_(param), aoi_polygon_(aoi_polygon) {
        CHECK(param_.resolution > 0.0);
        CHECK(param_.max_tolerance > 0.0);
        CHECK(param_.min_cut_distance > 0.0);
    }

    /**
     * Extract centerlines from the given road regions.
     */
    bool Extract(const Array<LandRegion>& road_regions,
                 const std::string& default_category) {
        if (road_regions.empty()) return false;

        LandRegionArrangement arrangement;
        arrangement.Arrange(road_regions);
        Array<LandRegion> regions;
        arrangement.MergeSameLevelRegions(&regions);
        if (regions.empty()) return false;

        ConstrctGraph(regions);
        ExtractRoads(road_regions, default_category);

        return true;
    }

    /**
     * The result centerline road kernels.
     */
    const Array<Array<RoadKernel>>& road_kernels() const {
        return road_kernels_;
    }

    /**
     * Return raw centerline roads.
     */
    const Array<Road>& raw_roads() const {
        return raw_roads_;
    }

    const Array<RSegment2D>& bad_connections() const {
        return bad_connections_;
    }

    const std::unordered_map<int, std::set<RSegment2D>>&
    region_cutlines() const {
        return region_cutlines_;
    }

private:
    /**
     * Merge adjacent short roads without category/width consistency checks.
     */
    static void MergeShortRoadKernels(double length_threshold,
                                      Array<RoadKernel>* kernels) {
        CHECK(length_threshold > 0.0);
        CHECK(kernels);

        if (kernels->empty()) return;

        Array<RoadKernel> merged_kernels;
        merged_kernels.push_back((*kernels)[0]);

        for (int i = 1; i < kernels->size(); ++i) {
            const RoadKernel& current = (*kernels)[i];
            RoadKernel& last = merged_kernels.back();
            double last_length = last.polyline.Length();
            double current_length = current.polyline.Length();

            if (last_length <= length_threshold ||
                current_length <= length_threshold) {
                std::string current_category = current.category;
                std::string last_category = last.category;

                // Special treat for crosswalks and driveways.
                bool donot_merge = false;
                if (road_util::IsCrosswalk(current_category) ||
                    road_util::IsDriveway(current_category)) {
                    if (last_length <= length_threshold) {
                        carto_polyline_util::MergeTwoAdjacentKernels(current,
                                                                     &last);
                        last.category = current_category;
                        continue;
                    }
                    donot_merge = true;
                }
                if (road_util::IsCrosswalk(last_category) ||
                    road_util::IsDriveway(last_category)) {
                    if (current_length <= length_threshold) {
                        carto_polyline_util::MergeTwoAdjacentKernels(current,
                                                                     &last);
                        last.category = last_category;
                        continue;
                    }
                    donot_merge = true;
                }
                if (!donot_merge) {
                    carto_polyline_util::MergeTwoAdjacentKernels(current, &last);
                    last.category =
                            last_length < current_length ? current_category
                                                         : last_category;
                    continue;
                }
            }
            merged_kernels.push_back(current);
        }

        *kernels = merged_kernels;
    }

    /**
     * Constrcut region graph from level-regions.
     */
    void ConstrctGraph(const Array<LandRegion>& level_regions) {
        graph_.clear();
        region_level_.clear();
        region_cutlines_.clear();

        for (const auto& r : level_regions) {
            Array<Array<RPoint2D>> face;
            for (auto b : r.polygon.boundaries()) {
                face.push_back(b.polygon.vertices());
            }
            int fid = graph_.InsertFace(face);
            if (fid != -1) {
                region_level_[fid] = r.level;
            }
        }

        // Decide region-to-region connections by the TOTAL length of their
        // shared boundary. For every shared edge we record the candidate region
        // pair (same seamless / same-level rules as before) and accumulate the
        // shared-boundary length. A pair is only connected (its cutlines
        // registered) when that total length reaches min_shared_length, so two
        // regions that merely touch along a tiny sliver are treated as NOT
        // connected.
        const double min_shared_length = param_.resolution;
        std::map<std::pair<int, int>, double> shared_length;
        std::map<std::pair<int, int>, Array<std::pair<int, RSegment2D>>>
                shared_cutlines;

        for (auto e : graph_.edges()) {
            int twin = graph_.TwinEdge(e.id);
            if (twin == -1) continue;

            RSegment2D cutline(graph_.source_point(e),
                               graph_.target_point(e));
            double length = Distance(graph_.source_point(e),
                                     graph_.target_point(e));

            Array<int> faces1, faces2;
            graph_.GetFacesOfEdge(e.id, &faces1);
            graph_.GetFacesOfEdge(twin, &faces2);

            auto add_candidate = [&](int f1, int f2) {
                if (f1 == f2) return;
                auto key = std::make_pair(std::min(f1, f2), std::max(f1, f2));
                // Each undirected edge contributes its length once; the cutline
                // segment is recorded for both traversal directions so each
                // region finds it on its own boundary.
                if (e.id < twin) shared_length[key] += length;
                shared_cutlines[key].emplace_back(f1, cutline);
                shared_cutlines[key].emplace_back(f2, cutline);
            };

            if (faces1.size() == 1 && faces2.size() == 1) {
                // A clean shared boundary edge (one region on each side) means
                // the two regions are seamlessly adjacent with no overlap, so
                // they may connect regardless of level -- e.g. a level-1
                // overpass ramp coming down to seamlessly join the level-0
                // road.
                // Stacked/overlapping regions do NOT share a clean boundary
                // edge, so they never reach here.
                add_candidate(faces1.front(), faces2.front());
            } else if (faces1.size() >= 1 && faces2.size() >= 1) {
                for (int f1 : faces1) {
                    for (int f2 : faces2) {
                        if (region_level_[f1] == region_level_[f2]) {
                            add_candidate(f1, f2);
                            break;
                        }
                    }
                }
            }
        }

        for (const auto& p : shared_cutlines) {
            if (shared_length[p.first] < min_shared_length) continue;
            for (const auto& fc : p.second) {
                region_cutlines_[fc.first].insert(fc.second);
            }
        }
    }

    /**
     * Extract roads from region-graph.
     */
    void ExtractRoads(const Array<LandRegion>& road_regions,
                      const std::string& default_category) {
        road_kernels_.clear();
        raw_roads_.clear();

        RoadCategorizer categorizer(road_regions);

        for (auto f : graph_.faces()) {
            const auto& region_cutlines = region_cutlines_[f.id];

            int level = region_level_[f.id];
            RMultiPolygon2D polygon;
            geometry::geo_graph_util::GetRawFacePolygon(graph_, f.id, &polygon);

            RMultiPolygon2D densified_polygon;
            for (const auto& b : polygon.boundaries()) {
                Array<RPoint2D> vertices;
                for (int i = 0; i < b.polygon.size(); ++i) {
                    RPolyline2D polyline;
                    polyline.push_back(b.polygon.vertex(i));
                    polyline.push_back(b.polygon.next_vertex(i));
                    RSegment2D e = b.polygon.edge(i);
                    RPolyline2D densified;
                    if (region_cutlines.find(e) == region_cutlines.end()) {
                        geometry::polyline_util::Densify(polyline,
                                                         param_.resolution,
                                                         &densified);
                        vertices.insert(densified);
                    } else {
                        vertices.insert(polyline);
                    }
                }
                RPolygon2D poly(vertices);
                if (poly.empty()) {
                    if (densified_polygon.empty()) break;
                    continue;
                }

                if (densified_polygon.empty()) {
                    densified_polygon.Insert(poly, true);
                } else {
                    densified_polygon.Insert(poly, false);
                }
            }

            geometry::TriangulationAxis2D<double> medial_axis;
            medial_axis.Construct(densified_polygon, region_cutlines);
            medial_axis.PruneBPR(3.0, param_.min_cut_distance);

            Array<RPolyline2D> roads;
            medial_axis.ToBranches(&roads);

            Array<Array<RoadKernel>> road_kernels;
            RoadCenterlineOptimizer optimizer(aoi_polygon_, param_.resolution,
                                              param_.max_tolerance);
            optimizer.Optimize(polygon, medial_axis);
            optimizer.ToRoadKernels(&road_kernels);
            bad_connections_.insert(optimizer.bad_connections());

            for (const auto& b : optimizer.branches()) {
                Road road;
                road.level = level;
                road.category = "raw_centerline";
                road.widths = b.raw_widths;
                road.polyline = b.raw_polyline;
                raw_roads_.push_back(road);
            }

            for (auto& kernels : road_kernels) {
                for (auto& s : kernels) {
                    s.category = default_category;
                    s.level = level;
                }
                categorizer.AssignCategory(kernels, &kernels);
                categorizer.ReassignSpecialCategories(10.0 * param_.resolution,
                                                      &kernels);

                carto_polyline_util::MergeAdjacentKernels(1.0, 4.0, &kernels);
                MergeShortRoadKernels(1.0, &kernels);
                for (auto& s : kernels) {
                    geometry::polyline_util::DouglasPeuckerSimplify(
                                s.polyline, 0.01, &s.polyline);
                }
                road_kernels_.push_back(kernels);
            }
        }
    }

    // Parameters for road centerline algorithm.
    Parameter param_;

    // Graph between level regions.
    geometry::GeoGraph<RPoint2D> graph_;

    // Level for each graph's region.
    std::unordered_map<int, int> region_level_;

    // Cutlines for each graph's region.
    std::unordered_map<int, std::set<RSegment2D>> region_cutlines_;

    // The output result.
    Array<Array<RoadKernel>> road_kernels_;

    // The extracted raw roads.
    Array<Road> raw_roads_;

    // Failed to find a correct connection.
    Array<RSegment2D> bad_connections_;

    // AOI (Area of Interest) polygon for centerline filtering.
    // Only centerlines falling within this polygon are saved; if aoi_polygon is
    // empty (no AOI specified), all centerlines will be saved.
    RMultiPolygon2D aoi_polygon_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CENTERLINE_EXTRACTOR_H_
