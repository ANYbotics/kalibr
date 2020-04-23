#ifndef ASLAM_SPLINES_B_SPLINE_EXPRESSIONS_HPP
#define ASLAM_SPLINES_B_SPLINE_EXPRESSIONS_HPP


#include <aslam_bsplines/BSplinePose.hpp>
#include <aslam_backend_expressions/TransformationExpression.hpp>
#include <aslam_backend/backend/backend.hpp>
#include <aslam_backend_expressions/RotationExpression.hpp>
#include <aslam_backend_expressions/EuclideanExpression.hpp>
#include <aslam_backend_expressions/VectorExpression.hpp>
#include <aslam_backend_expressions/TransformationExpressionNode.hpp>
#include <aslam_backend_expressions/RotationExpressionNode.hpp>
#include <aslam_backend_expressions/EuclideanExpressionNode.hpp>
#include <aslam_backend_expressions/VectorExpressionNode.hpp>
#include <aslam_backend_expressions/DesignVariableVector.hpp>
#include <aslam_backend_expressions/ScalarExpression.hpp>

namespace aslam {
    namespace splines {
        class BSplinePoseDesignVariable;

        // aslam::backend::TransformationExpression transformation(double tk);
        class BSplineTransformationExpressionNode : public aslam::backend::TransformationExpressionNode
        {
        public:
            BSplineTransformationExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineTransformationExpressionNode();

        protected:
            virtual Eigen::Matrix4d toTransformationMatrixImplementation();
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;
        };

        // aslam::backend::TransformationExpression transformation(double tk);
        class BSplineRotationExpressionNode : public aslam::backend::RotationExpressionNode
        {
        public:
            BSplineRotationExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineRotationExpressionNode();

        protected:
            virtual Eigen::Matrix3d toRotationMatrixImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;

        };


        // aslam::backend::EuclideanExpression position(double tk);
        // aslam::backend::TransformationExpression transformation(double tk);
        class BSplinePositionExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
            BSplinePositionExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplinePositionExpressionNode();

        protected:
            virtual Eigen::Vector3d toEuclideanImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;

        };

        // Fabio:
        class BSplineVelocityExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
            BSplineVelocityExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineVelocityExpressionNode();

        protected:
            virtual Eigen::Vector3d toEuclideanImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;

        };


        // aslam::backend::VectorExpression<3> linearVelocity(double tk);

        class BSplineAccelerationExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
            BSplineAccelerationExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineAccelerationExpressionNode();

        protected:
            virtual Eigen::Vector3d toEuclideanImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;

        };

    class BSplineAccelerationBodyFrameExpressionNode :
      public aslam::backend::EuclideanExpressionNode {
    public:
        BSplineAccelerationBodyFrameExpressionNode(aslam_bsplines::BSplinePose*
          spline, const std::vector<aslam::backend::DesignVariable*>&
          designVariables, double time);
        virtual ~BSplineAccelerationBodyFrameExpressionNode();

    protected:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(
          aslam::backend::JacobianContainer& outJacobians) const;
        virtual void evaluateJacobiansImplementation(
          aslam::backend::JacobianContainer& outJacobians,
          const Eigen::MatrixXd& applyChainRule) const;
        virtual void getDesignVariablesImplementation(
          aslam::backend::DesignVariable::set_t& designVariables) const;

        aslam_bsplines::BSplinePose* _spline;
        std::vector<aslam::backend::DesignVariable*> _designVariables;
        double _time;

    };

        class BSplineAngularVelocityBodyFrameExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
            BSplineAngularVelocityBodyFrameExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineAngularVelocityBodyFrameExpressionNode();

        protected:
            virtual Eigen::Vector3d toEuclideanImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSplinePose * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;

        };


        class BSplineAngularAccelerationBodyFrameExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
        BSplineAngularAccelerationBodyFrameExpressionNode(aslam_bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
        virtual ~BSplineAngularAccelerationBodyFrameExpressionNode();
         
        protected:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;
         
        aslam_bsplines::BSplinePose * _spline;
        std::vector<aslam::backend::DesignVariable *> _designVariables;
        double _time;
         
        };

        template<int DIM>
        class BSplineVectorExpressionNode : public aslam::backend::VectorExpressionNode<DIM>
        {
        public:
            typedef typename aslam::backend::VectorExpressionNode<DIM>::vector_t vector_t;
            BSplineVectorExpressionNode(aslam_bsplines::BSpline * spline, int derivativeOrder, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time);
            virtual ~BSplineVectorExpressionNode();

        protected:
            virtual vector_t evaluateImplementation() const;

            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSpline * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;
            int _derivativeOrder;

        };    
        
        // aslam::backend::TransformationExpression transformation(double tk);
        class TransformationTimeOffsetExpressionNode : public aslam::backend::TransformationExpressionNode
        {
        public:    
        
            TransformationTimeOffsetExpressionNode(BSplinePoseDesignVariable * bspline, const aslam::backend::ScalarExpression & time,
            										double bufferTmin = 0, double bufferTmax = 0);
            virtual ~TransformationTimeOffsetExpressionNode();

        protected:
            virtual Eigen::Matrix4d toTransformationMatrixImplementation();
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;


            BSplinePoseDesignVariable * _spline;
            aslam::backend::ScalarExpression  _time;

            int _bufferLeft;
            int _bufferRight;
            double _bufferTmin;
            double _bufferTmax;
            Eigen::VectorXi _localCoefficientIndices;

        };

        class BSplineEuclideanExpressionNode : public aslam::backend::EuclideanExpressionNode
        {
        public:
            BSplineEuclideanExpressionNode(aslam_bsplines::BSpline * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time, int derivativeOrder);
            virtual ~BSplineEuclideanExpressionNode();

        protected:
            virtual Eigen::Vector3d toEuclideanImplementation() const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const;
            virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
            virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const;

            aslam_bsplines::BSpline * _spline;
            std::vector<aslam::backend::DesignVariable *> _designVariables;
            double _time;
            int _order;
        };



    } // namespace splines
} // namespace aslam

#include "implementation/BSplineExpressions.hpp"

#endif /* ASLAM_SPLINES_B_SPLINE_EXPRESSIONS_HPP */
