#include <aslam_backend_expressions/MatrixExpression.hpp>
#include <aslam_backend_expressions/MatrixExpressionNode.hpp>
#include <aslam_backend_expressions/EuclideanExpressionNode.hpp>
#include <sm_boost/null_deleter.hpp>

namespace aslam {
namespace backend {

/// \brief the base case is to initialize an expression from a design variable.
MatrixExpression::MatrixExpression(MatrixExpressionNode * rotationDesignVariable)
{
	_root.reset(rotationDesignVariable,sm::null_deleter());
	SM_ASSERT_TRUE(Exception, _root.get() != NULL,
			"It is illegal to initialized a rotation expression with a null node");
}

MatrixExpression::MatrixExpression()
{
	// 0
}

MatrixExpression::~MatrixExpression()
{
	// 0
}

MatrixExpression::MatrixExpression(boost::shared_ptr<MatrixExpressionNode> node) :
    		  _root(node)
{
	SM_ASSERT_TRUE(Exception, _root.get() != NULL,
			"It is illegal to initialized a matrix transformation expression with a null node");
}

/// \brief Evaluate the full transformation matrix.
Eigen::Matrix3d MatrixExpression::toMatrix3x3()
{
	return _root->toMatrix3x3();
}

/// \brief Evaluate the Jacobians
void MatrixExpression::evaluateJacobians(JacobianContainer & outJacobians) const
{
	_root->evaluateJacobians(outJacobians);
}

EuclideanExpression MatrixExpression::operator*(const EuclideanExpression & p) const
{
	boost::shared_ptr<EuclideanExpressionNode> newRoot(
			new EuclideanExpressionNodeMatrixMultiply(_root, p._root));	// ##
	return EuclideanExpression(newRoot);

}

void MatrixExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
{
	return _root->getDesignVariables(designVariables);
}

} // namespace backend
} // namespace aslam
