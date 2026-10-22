////
//// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
////
//// This file is part of the Code Library.
////

//#ifndef CODELIBRARY_POINT_CLOUD_ITERATIVE_CLOSEST_POINT_H_
//#define CODELIBRARY_POINT_CLOUD_ITERATIVE_CLOSEST_POINT_H_

//#include <utility>

//#include "codelibrary/base/numbers.h"
//#include "codelibrary/geometry/transform_3d.h"
//#include "codelibrary/math/matrix/isometry.h"
//#include "codelibrary/math/matrix/matrix.h"
//#include "codelibrary/math/vector.h"
//#include "codelibrary/point_cloud/principal_component_analysis_3d.h"
//#include "codelibrary/util/tree/kd_tree.h"

//namespace cl {
//namespace point_cloud {

///**
// * Iterative closest point (ICP) is an algorithm employed to minimize the
// * difference between two point clouds.
// *
// * The following code is heavily inspired by the small_gicp library.
// *
// * Reference:
// *   Segal, Aleksandr, Dirk Haehnel, and Sebastian Thrun. "Generalized-icp",
// *   Robotics: science and systems. Vol.2, No.4, 2009.
// */
//template <class Point>
//class IterativeClosestPoint {
//    using KDTree = KDTree<Point>;

//public:
//    /**
//     * Status code indicating the result of the optimization in one step.
//     */
//    enum class IterationState {
//        CONVERGED = 0, // Optimization has converged.
//        SUCCESS = 1,   // Optimization was successful but will continue in the
//                       // next step.
//        FAILED = 2     // Optimization has failed.
//    };

//    /**
//     * Constructor for the IterativeClosestPoint class.
//     *
//     * Paramters:
//     *   num_neighbors               - Number of neighbor points used for
//     *                                 covariance estimation.
//     *   max_correspondence_distance - Maximum correspondence distance between
//     *                                 points (e.g., triming threshold).
//     */
//    IterativeClosestPoint(double max_correspondence_distance = 1.0,
//                          int n_neighbors = 10)
//        : max_correspondence_distance_(max_correspondence_distance),
//          n_neighbors_(n_neighbors) {
//        CHECK(max_correspondence_distance_ > 0.0);
//        CHECK(n_neighbors >= 5);
//    }

//    /**
//     * Align source points to the target points.
//     * Assume that the source and target points are stored in KD trees.
//     */
//    IterationState Align(int max_iterations, RMatrix4* transformation) {
//        CHECK(source_) << "Call Reset() first.";
//        CHECK(target_) << "Call Reset() first.";
//        CHECK(max_iterations > 0);
//        CHECK(transformation);

//        while (max_iterations--) {
//            IterationState s = SingleStep(transformation);
//            if (s == IterationState::CONVERGED ||
//                s == IterationState::FAILED) return s;
//        }
//        return IterationState::SUCCESS;
//    }

//    /**
//     * Reset ICP problem and the initial transformation.
//     */
//    void Reset(const KDTree& source, const KDTree& target,
//               RMatrix4* transformation) {
//        CHECK(transformation);

//        source_ = &source;
//        target_ = &target;
//        CompuateCovariances(source, &source_covariances_);
//        CompuateCovariances(target, &target_covariances_);

//        lambda_ = 1e-3;
//        transformation->SetIdentity();
//    }

//    /**
//     * Single step of ICP.
//     *
//     * Need to call Initialize() first.
//     */
//    IterationState SingleStep(RMatrix4* transformation) {
//        CHECK(source_);
//        CHECK(target_);
//        CHECK(source_->size() == source_covariances_.size());
//        CHECK(target_->size() == target_covariances_.size());
//        CHECK(transformation);

//        RMatrix4& t = *transformation;

//        Matrix<double> h(6, 6);
//        RVector b(6);
//        Matrix<double> jacobi(4, 6);

//        Array<std::pair<int, int>> correspondences;
//        RMatrix4 mahalanobis;
//        RVector residual(4);

//        const Array<Point>& source_points = source_->points();
//        const Array<Point>& target_points = target_->points();

//        int max_inner_iterations = 10;
//        while (max_inner_iterations--) {
//            h.Fill(0.0);
//            b.fill(0.0);
//            double e = 0.0;
//            for (int i = 0; i < source_points.size(); ++i) {
//                Point p = source_points[i];
//                Point p_trans = geometry::Rigid(p, t);

//                int j = target_->FindNearestIndex(p_trans);
//                Point q = target_points[j];

//                // Discard point pairs that are too far apart.
//                if (Distance(p, q) > max_correspondence_distance_)
//                    continue;

//                correspondences.emplace_back(i, j);

//                ComputeResidual(t, i, j, p_trans, q, &mahalanobis, &residual);
//                jacobi.Fill(0.0);
//                jacobi.block<3, 3>(0, 0) = t.linear() *
//                                           se3::Skew(RVector3(p.x, p.y, p.z));
//                jacobi.block<3, 3>(0, 3) = -t.linear();

//                h += jacobi.Transpose() * mahalanobis * jacobi;
//                b += jacobi.Transpose() * mahalanobis * residual;
//                e += 0.5 * residual.Transpose() * mahalanobis * residual;
//            }

//            // Optimization.
//            RVector delta = (h + lambda_ * Matrix<double, 6, 6>::Identity()).
//                             ldlt().solve(-b);
//            RIsometry3 new_t = t * se3::Exp(delta);
//            double new_e = 0.0;
//            for (const auto& c : correspondences) {
//                int i = c.first;
//                int j = c.second;
//                Point p = geometry::Rigid(source_points[i], new_t);
//                Point q = target_points[j];
//                ComputeResidual(new_t, i, j, p, q, &mahalanobis, &residual);
//                new_e += 0.5 * residual.Transpose() * mahalanobis * residual;
//            }

//            if (new_e <= e) {
//                // Error decreased, decrease lambda.
//                t = new_t;

//                if (delta.head<3>().norm() <= rotation_eps_ &&
//                    delta.tail<3>().norm() <= translation_eps_) {
//                    return IterationState::CONVERGED;
//                }
//                lambda_ /= dumping_factor_;
//                return IterationState::SUCCESS;
//            }

//            // Failed to decrease error, increase lambda.
//            lambda_ *= dumping_factor_;
//        }

//        return IterationState::FAILED;
//    }

//private:
//    /**
//     * Compute a regularized version of the covariance matrix for each point
//     * based on its nearest neighbors in the KD tree.
//     *
//     * These matrices are used in the Generalized-ICP algorithm to improve point
//     * alignment results.
//     */
//    void CompuateCovariances(const KDTree& kd_tree,
//                             Array<RMatrix4>* covariances) {
//        const auto& points = kd_tree.points();
//        covariances->resize(points.size());
//        Array<Point> neighbors;

//        for (int i = 0; i < points.size(); ++i) {
//            kd_tree.FindKNearestNeighbors(points[i], n_neighbors_, &neighbors);
//            PrincipalComponentAnalysis3D<double> pca(neighbors);
//            const Array<RVector3D>& eigenvectors = pca.eigenvectors();

//            RMatrix4 cov;
//            RVector3D diag(1e-3, 1.0, 1.0);
//            cov.block<3, 3>(0, 0) = eigenvectors * diag *
//                                    eigenvectors.transpose();
//            (*covariances)[i] = cov;
//        }
//    }

//    /**
//     * Used for ICP optimization.
//     */
//    void ComputeResidual(const RIsometry3& t, int i, int j,
//                         const Point& p, const Point& q,
//                         RMatrix4* mahalanobis, RVector4* residual) const {
//        const RMatrix4 rcr = target_covariances_[j] +
//                             t.matrix() * source_covariances_[i] *
//                             t.matrix().transpose();
//        mahalanobis->setZero();
//        mahalanobis->block<3, 3>(0, 0) = rcr.block<3, 3>(0, 0).inverse();
//        *residual << q.x - p.x, q.y - p.y, q.z - p.z, 0.0;
//    }

//    // Maximum correspondence distance between points.
//    double max_correspondence_distance_ = 0.0;
//    // Number of nearest neighbors.
//    int n_neighbors_ = 0;
//    // Used in optimization.
//    double lambda_ = 1e-3;
//    // Increasing this makes optimization slow but stable.
//    double dumping_factor_ = 10.0;

//    // Termination criteria.
//    double translation_eps_ = 1e-3;
//    double rotation_eps_ = 0.1 * numbers::pi / 180.0;

//    // Covariances matrix for source and target point cloud.
//    Array<RMatrix4> source_covariances_;
//    Array<RMatrix4> target_covariances_;

//    // Pointer to the source and target point clouds.
//    const KDTree* source_ = nullptr;
//    const KDTree* target_ = nullptr;
//};

//} // namespace point_cloud
//} // namespace cl

//#endif // CODELIBRARY_POINT_CLOUD_ITERATIVE_CLOSEST_POINT_H_
