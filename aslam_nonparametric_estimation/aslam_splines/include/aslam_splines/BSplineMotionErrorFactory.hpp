#ifndef _BSPLINEMOTIONERRORFACTORY_H_
#define _BSPLINEMOTIONERRORFACTORY_H_

#include <aslam_backend/backend/ErrorTerm.hpp>
#include <aslam_backend/backend/OptimizationProblem.hpp>
#include <aslam_bsplines/BSplinePose.hpp>
#include <aslam_splines/BSplinePoseDesignVariable.hpp>
#include <aslam_backend/backend/MarginalizationPriorErrorTerm.hpp>


namespace aslam {
namespace backend {

template<class BSplineDesignVariable>
void addMotionErrorTerms(OptimizationProblemBase& problem, BSplineDesignVariable& spline, const Eigen::MatrixXd& W, unsigned int errorTermOrder);

} // namespace backend
} // namespace aslam

#include "implementation/BSplineMotionErrorFactory.hpp"

#endif /* _BSPLINEMOTIONERRORFACTORY_H_ */
