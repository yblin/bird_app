//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_
#define CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_

#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/util/rtree_2d.h"
#include "codelibrary/gis/carto_polyline.h"
#include "codelibrary/gis/land_region.h"
#include "codelibrary/gis/road_system/road_util.h"

namespace cl {
namespace gis {

/**
 * Assigning categories to road polylines based on associated landcover data.
 */
class RoadCategorizer {
public:
    using Road = CartoPolyline2D;
    using RoadKernel = CartoPolylineKernel2D;

    RoadCategorizer() = default;

    explicit RoadCategorizer(const Array<LandRegion>& land_regions) {
        ResetLandRegions(land_regions);
    }

    RoadCategorizer(const RoadCategorizer&) = delete;

    RoadCategorizer& operator=(const RoadCategorizer&) = delete;

    /**
     * Reset land regions for category assigning.
     */
    void ResetLandRegions(const Array<LandRegion>& land_regions) {
        this->clear();

        for (auto& r : land_regions) {
            level_land_regions_[r.level].push_back(r);
        }

        for (auto& r : level_land_regions_) {
            Array<RBox2D> boxes;
            for (const auto& r1 : r.second) {
                boxes.push_back(r1.polygon.bounding_box());
            }
            level_rtrees_[r.first].reset(new geometry::RTree2D<double>());
            auto rtree = level_rtrees_[r.first].get();
            rtree->Construct(boxes);
        }
    }

    /**
     * Check whether the given road intersects with the polygon associated with
     * the specified category.
     */
    bool HasCategory(const Road& road, const std::string& category) const {
        if (level_rtrees_.find(road.level) == level_rtrees_.end())
            return false;

        auto rtree = level_rtrees_.at(road.level).get();
        const auto& regions = level_land_regions_.at(road.level);

        RBox2D box(road.polyline.begin(), road.polyline.end());
        Array<int> entries;
        rtree->Find(box, &entries);
        for (int i : entries) {
            if (regions[i].category != category) continue;
            if (geometry::Intersect(road.polyline, regions[i].polygon))
                return true;
        }
        return false;
    }

    /**
     * Split a road (composed of multiple road kernels) into multiple road
     * kernels, where each kernel is assigned a category corresponding to the
     * landcover polygon that covers that kernel.
     */
    void AssignCategory(const Array<RoadKernel>& kernels,
                        Array<RoadKernel>* kernels_with_category) const {
        CHECK(kernels_with_category);

        if (kernels.empty()) {
            kernels_with_category->clear();
            return;
        }

        int level = kernels.front().level;
        for (const auto& s : kernels) {
            CHECK(level == s.level);
        }

        if (level_rtrees_.find(level) == level_rtrees_.end()) return;

        auto rtree = level_rtrees_.at(level).get();
        const auto& regions = level_land_regions_.at(level);

        RBox2D box;
        for (const auto& s : kernels) {
            RBox2D box1(s.polyline.begin(), s.polyline.end());
            box.Union(box1);
        }
        *kernels_with_category = kernels;

        Array<int> entries;
        rtree->Find(box, &entries);

        for (int i : entries) {
            Array<RoadKernel> segs, new_kernels;
            for (const auto& s : *kernels_with_category) {
                AssignCategory(s, regions[i], &segs);
                new_kernels.insert(segs);
            }
            *kernels_with_category = new_kernels;
        }
    }

    /**
     * Reassign the short side runs flanking every special block. This is a
     * convenience entry point that applies each special-category pass in
     * turn: crosswalk first (using the caller-supplied `threshold`), then
     * driveway (which derives its own threshold from each driveway block's
     * length). See ReassignCrosswalkCategory and ReassignDrivewayCategory.
     */
    void ReassignSpecialCategories(double threshold,
                                   Array<RoadKernel>* kernels) {
        ReassignCrosswalkCategory(threshold, kernels);
        ReassignDrivewayCategory(kernels);
    }

    void clear() {
        level_land_regions_.clear();
        level_rtrees_.clear();
    }

private:
    /**
     * Split a single road kernel into multiple kernels based on the specific
     * land region, and assign the polygon's corresponding category to the
     * overlapping parts.
     */
    void AssignCategory(const RoadKernel& kernel, const LandRegion& region,
                        Array<RoadKernel>* kernels) const {
        CHECK(kernels);

        kernels->clear();

        if (kernel.polyline.empty()) return;

        const auto& polygon = region.polygon;
        const std::string& category = region.category;
        if (kernel.category == category) {
            kernels->push_back(kernel);
            return;
        }

        using Segment = typename RPolyline2D::Segment;
        const int n = kernel.polyline.size();

        RPolyline2D poly1, poly2;
        RoadKernel k_in = kernel;
        k_in.category = category;
        RoadKernel k_out = kernel;

        for (int i = 0; i + 1 < n; ++i) {
            if (kernel.polyline[i] == kernel.polyline[i + 1]) continue;

            const Segment& seg = kernel.polyline.segment(i);
            Array<RPoint2D> intersection;
            geometry::Intersect(seg, polygon, &intersection);
            CHECK(intersection.size() % 2 == 0);

            const RPoint2D& prev = kernel.polyline[i];
            const RPoint2D& next = kernel.polyline[i + 1];

            if (!intersection.empty()) {
                if (prev != intersection.front()) {
                    if (poly2.empty()) poly2.push_back(prev);
                    poly2.push_back(intersection.front());

                    if (!poly1.empty()) {
                        k_in.polyline = poly1;
                        kernels->push_back(k_in);
                        poly1.clear();
                    }
                }

                for (int j = 0; j < intersection.size(); ++j) {
                    if (j % 2 == 0) {
                        if (!poly2.empty()) {
                            k_out.polyline = poly2;
                            kernels->push_back(k_out);
                            poly2.clear();
                        }

                        if (intersection[j] != intersection[j + 1]) {
                            if (poly1.empty()) {
                                poly1.push_back(intersection[j]);
                            }
                            poly1.push_back(intersection[j + 1]);
                        }
                    } else {
                        if (j + 1 == intersection.size()) break;
                        if (!poly1.empty()) {
                            k_in.polyline = poly1;
                            kernels->push_back(k_in);
                            poly1.clear();
                        }

                        if (intersection[j] != intersection[j + 1]) {
                            if (poly2.empty()) {
                                poly2.push_back(intersection[j]);
                            }
                            poly2.push_back(intersection[j + 1]);
                        }
                    }
                }

                if (next != intersection.back()) {
                    if (poly2.empty()) poly2.push_back(intersection.back());
                    poly2.push_back(next);

                    if (!poly1.empty()) {
                        k_in.polyline = poly1;
                        kernels->push_back(k_in);
                        poly1.clear();
                    }
                }
            } else {
                if (poly2.empty()) poly2.push_back(prev);
                poly2.push_back(next);
                if (!poly1.empty()) {
                    k_in.polyline = poly1;
                    kernels->push_back(k_in);
                    poly1.clear();
                }
            }
        }

        CHECK(poly1.empty() || poly2.empty());
        if (!poly1.empty()) {
            k_in.polyline = poly1;
            kernels->push_back(k_in);
        }
        if (!poly2.empty()) {
            k_out.polyline = poly2;
            kernels->push_back(k_out);
        }

        int n_kernels = 0;
        for (RoadKernel& kernel : *kernels) {
            kernel.polyline.Trim();
            if (kernel.polyline.size() >= 2) {
                (*kernels)[n_kernels++] = kernel;
            }
        }
        kernels->resize(n_kernels);

        // In case where the polyline degenerates into a point.
        if (kernels->empty()) {
            if (geometry::Intersect(kernel.polyline.front(), polygon)) {
                kernels->push_back(kernel);
                kernels->back().category = category;
            }
        }
    }

    /**
     * Reassign short side runs (e.g. sidewalk) that sit immediately next to
     * a crosswalk block so they take on the crosswalk's category.
     *
     * This function does NOT merge or combine kernels -- the kernel array
     * size is unchanged, polylines are untouched. It only rewrites the
     * `.category` string of selected kernels.
     *
     * Algorithm: every maximal run of consecutive non-crosswalk kernels is
     * treated as a single unit, its length being the sum of its kernels'
     * lengths. For each contiguous crosswalk block, the run immediately to
     * its left and the run immediately to its right are considered
     * independently: if a run's total length is strictly less than
     * `threshold`, every kernel in that run is relabeled with the category
     * of the crosswalk kernel adjacent to the run (block_start's category
     * for a left run, block_end's category for a right run).
     *
     * Runs are bounded by adjacent crosswalk blocks (or by the start/end of
     * the kernels array), so a kernel separated from the current block by
     * another crosswalk block is never affected by the current block.
     *
     * `threshold` is an absolute length in the same units as
     * RoadKernel::polyline.Length() (e.g. meters); it is NOT relative to
     * the crosswalk block's length.
     *
     * Example: [s:3, s:8, s:5, crosswalk:10] with threshold=10
     *   left run = [s:3, s:8, s:5], total = 16
     *   16 < 10 is false -> the three s kernels keep "sidewalk".
     *
     * Example: [s:3, s:3, s:3, crosswalk:10] with threshold=10
     *   left run total = 9, 9 < 10 -> all three s kernels are relabeled
     *   "crosswalk".
     *
     * Example: [s:5, crosswalk:10, s:2, crosswalk:8, s:7] with threshold=6
     *   crosswalk@1: left run = [s:5] (5<6 relabel), right run = [s:2] (2<6
     *          relabel with crosswalk@1's category).
     *   crosswalk@3: left run = [s:2] -- the is_special flags are captured
     *          before any relabeling, so s:2 is still seen as non-special and
     *          is eligible to be relabeled again, this time with crosswalk@3's
     *          category. The later block wins. If both crosswalk blocks
     *          share the same category string this is a no-op.
     *          right run = [s:7] (7<6 false, kept as sidewalk).
     */
    void ReassignCrosswalkCategory(double threshold,
                                   Array<RoadKernel>* kernels) {
        CHECK(kernels);
        const int n = kernels->size();
        if (n == 0) return;

        // Cache (length, is_special) per kernel. The is_special flags are
        // captured BEFORE any relabeling so that run boundaries stay stable
        // across the outer loop -- relabeled kernels are still treated as
        // non-special in subsequent iterations.
        Array<bool> is_special(n, false);
        Array<double> length(n, 0.0);
        for (int i = 0; i < n; ++i) {
            length[i] = (*kernels)[i].polyline.Length();
            const std::string& cat = (*kernels)[i].category;
            is_special[i] = road_util::IsCrosswalk(cat);
        }

        // Walk every contiguous special block from left to right.
        int i = 0;
        while (i < n) {
            if (!is_special[i]) { ++i; continue; }

            // Collect the current contiguous block of special kernels.
            const int block_start = i;
            while (i < n && is_special[i]) ++i;
            const int block_end = i - 1;

            // ---- Left run -------------------------------------------------
            // Walk backwards across the maximal run of non-special kernels
            // immediately before the block. The loop stops at the previous
            // special block or at the array start.
            int left_run_start = block_start;
            double left_run_len = 0.0;
            while (left_run_start > 0 && !is_special[left_run_start - 1]) {
                left_run_len += length[left_run_start - 1];
                --left_run_start;
            }
            // All-or-nothing per run: only relabel when the whole run is
            // strictly shorter than the threshold. Left run inherits the
            // category of the kernel it is adjacent to (block_start).
            if (left_run_start < block_start && left_run_len < threshold) {
                const std::string& cat = (*kernels)[block_start].category;
                for (int k = left_run_start; k < block_start; ++k) {
                    (*kernels)[k].category = cat;
                }
            }

            // ---- Right run ------------------------------------------------
            // Symmetric: walk forwards across the run of non-special kernels
            // immediately after the block, bounded by the next special block
            // or the array end. Right run inherits the category of the
            // kernel it is adjacent to (block_end).
            int right_run_end = block_end + 1;
            double right_run_len = 0.0;
            while (right_run_end < n && !is_special[right_run_end]) {
                right_run_len += length[right_run_end];
                ++right_run_end;
            }
            if (right_run_end > block_end + 1 && right_run_len < threshold) {
                const std::string& cat = (*kernels)[block_end].category;
                for (int k = block_end + 1; k < right_run_end; ++k) {
                    (*kernels)[k].category = cat;
                }
            }
        }
    }

    /**
     * Reassign short side runs that sit immediately next to a driveway block
     * so they take on the driveway's category.
     *
     * Same neighbour-run algorithm as ReassignSpecialCategories, with one
     * difference: there is no caller-supplied threshold. The threshold for a
     * given driveway block is the block's own length -- the total length of
     * the driveway kernels that make up the block. A flanking run is relabeled
     * only when it is strictly shorter than the driveway it abuts, so the run
     * is absorbed only if the adjacent driveway is at least as long as it.
     *
     * Example: [s:3, driveway:4, driveway:4, s:5] (driveway block length = 8)
     *   left run = [s:3], 3 < 8 -> relabeled "driveway".
     *   right run = [s:5], 5 < 8 -> relabeled "driveway".
     *
     * Example: [s:10, driveway:4, s:2] (driveway block length = 4)
     *   left run = [s:10], 10 < 4 is false -> kept as sidewalk.
     *   right run = [s:2], 2 < 4 -> relabeled "driveway".
     */
    void ReassignDrivewayCategory(Array<RoadKernel>* kernels) {
        CHECK(kernels);
        const int n = kernels->size();
        if (n == 0) return;

        // Cache (length, is_driveway) per kernel. The is_driveway flags are
        // captured BEFORE any relabeling so that run boundaries stay stable
        // across the outer loop -- relabeled kernels are still treated as
        // non-driveway in subsequent iterations.
        Array<bool> is_driveway(n, false);
        Array<double> length(n, 0.0);
        for (int i = 0; i < n; ++i) {
            length[i] = (*kernels)[i].polyline.Length();
            is_driveway[i] = road_util::IsDriveway((*kernels)[i].category);
        }

        // Walk every contiguous driveway block from left to right.
        int i = 0;
        while (i < n) {
            if (!is_driveway[i]) { ++i; continue; }

            // Collect the current contiguous block of driveway kernels and
            // accumulate its total length, which is the threshold to use.
            const int block_start = i;
            double threshold = 0.0;
            while (i < n && is_driveway[i]) {
                threshold += length[i];
                ++i;
            }
            const int block_end = i - 1;

            // ---- Left run -------------------------------------------------
            // Walk backwards across the maximal run of non-driveway kernels
            // immediately before the block, bounded by the previous driveway
            // block or the array start.
            int left_run_start = block_start;
            double left_run_len = 0.0;
            while (left_run_start > 0 && !is_driveway[left_run_start - 1]) {
                left_run_len += length[left_run_start - 1];
                --left_run_start;
            }
            if (left_run_start < block_start && left_run_len < threshold) {
                const std::string& cat = (*kernels)[block_start].category;
                for (int k = left_run_start; k < block_start; ++k) {
                    (*kernels)[k].category = cat;
                }
            }

            // ---- Right run ------------------------------------------------
            int right_run_end = block_end + 1;
            double right_run_len = 0.0;
            while (right_run_end < n && !is_driveway[right_run_end]) {
                right_run_len += length[right_run_end];
                ++right_run_end;
            }
            if (right_run_end > block_end + 1 && right_run_len < threshold) {
                const std::string& cat = (*kernels)[block_end].category;
                for (int k = block_end + 1; k < right_run_end; ++k) {
                    (*kernels)[k].category = cat;
                }
            }
        }
    }

    // Land regions for each level.
    std::map<int, Array<LandRegion>> level_land_regions_;
    // RTree for each level.
    std::map<int, std::unique_ptr<geometry::RTree2D<double>>> level_rtrees_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_ROAD_SYSTEM_ROAD_CATEGORIZER_H_
