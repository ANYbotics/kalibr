#ifndef ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP
#define ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP

#include <aslam_backend_expressions/DesignVariableMappedVector.hpp>
#include <aslam_bsplines/BSpline.hpp>
#include <Eigen/StdVector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam_backend_expressions/TransformationExpression.hpp>
#include <aslam_backend_expressions/RotationExpression.hpp>
#include <aslam_backend_expressions/EuclideanExpression.hpp>
#include <aslam_backend_expressions/VectorExpression.hpp>
#include <aslam_backend_expressions/VectorExpressionNode.hpp>
#include "BSplineExpressions.hpp"
namespace aslam {
    namespace splines {
        

        template<int DIM>
        class BSplineDesignVariable
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
            enum {
                Dimension = DIM
            };

            typedef aslam::backend::DesignVariableMappedVector<Dimension> dv_t;

            /// \brief this guy takes a copy.
            BSplineDesignVariable(const aslam_bsplines::BSpline & bspline);
      
            virtual ~BSplineDesignVariable();

            /// \brief get the spline.
            const aslam_bsplines::BSpline & spline();
      
            /// \brief get an expression
            aslam::backend::VectorExpression<DIM> toExpression(double time, int derivativeOrder);

            size_t numDesignVariables();
            aslam::backend::DesignVariableMappedVector<DIM> * designVariable(size_t i);
      
            std::vector<aslam::backend::DesignVariable *> getDesignVariables(double time) const;

            // Fabio:
            // add one Segment at the end of the PoseSpline
            void addSegment(double t, const Eigen::VectorXd & p);
            void addSegment2(double t, const Eigen::VectorXd & p, double lambda);
            void removeSegment();

        protected:
            /// \brief the internal spline.
            aslam_bsplines::BSpline _bspline;

            /// \brief the vector of design variables.
            boost::ptr_vector< dv_t > _designVariables;
      
        };
    
    } // namespace splines
} // namespace aslam

#include "implementation/BSplineDesignVariable.hpp"

#endif /* ASLAM_SPLINES_BSPLINE_DESIGN_VARIABLE_HPP */
