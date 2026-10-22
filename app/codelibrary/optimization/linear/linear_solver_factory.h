//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.

#ifndef CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_FACTORY_H_
#define CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_FACTORY_H_

#include <memory>

#include "codelibrary/optimization/linear/cgnr_solver.h"

namespace cl {
namespace optimization {

class LinearSolverFactory {
public:
    static LinearSolverFactory* GetInstance() {
        static LinearSolverFactory factory;
        return &factory;
    }

    /**
     * Create a linear solver by solver type.
     */
    std::unique_ptr<SparseLinearSolver>
    Create(LinearSolverType type, const LinearSolver::Options& options) {
        switch (type) {
        case CGNR:
            return std::make_unique<CGNRSolver>(options);

        default:
            LOG(FATAL) << "Unknown linear solver type :" << type;
            return nullptr;
        }
    }
};

} // namespace optimization
} // namespace cl

#endif // CODELIBRARY_OPTIMIZATION_LINEAR_LINEAR_SOLVER_FACTORY_H_
