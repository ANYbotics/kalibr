#ifndef KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP
#define KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP

#include <aslam_backend/backend/ErrorTerm.hpp>
#include <aslam_backend_expressions/EuclideanExpression.hpp>
#include <aslam_backend_expressions/MatrixBasic.hpp>
#include <aslam_backend_expressions/MatrixExpression.hpp>
#include <aslam_backend_expressions/RotationExpression.hpp>

#include <kalibr/EuclideanError.hpp>

namespace kalibr {

class AccelerometerError : public EuclideanError {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// error = C_b_w(g - alpha) + bias - measurement;
	AccelerometerError(const Eigen::Vector3d & measurement,
			const Eigen::Matrix3d & invR,
			const aslam::backend::RotationExpression & C_b_w,
			const aslam::backend::EuclideanExpression & acceleration_w,
			const aslam::backend::EuclideanExpression & bias,
			const aslam::backend::EuclideanExpression & g_w);
	virtual ~AccelerometerError();
};


class AccelerometerErrorEccentric : public EuclideanError {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// error = C_b_w(g - alpha) + bias - measurement;
	AccelerometerErrorEccentric(
			const Eigen::Vector3d & measurement,
			const Eigen::Matrix3d & invR,
			const aslam::backend::MatrixExpression & M,
			const aslam::backend::RotationExpression & C_b_w,
			const aslam::backend::EuclideanExpression & acceleration_w,
			const aslam::backend::EuclideanExpression & angularVelocity_b,
			const aslam::backend::EuclideanExpression & angularAcceleration_b,
			const aslam::backend::RotationExpression & C_i_b,
			const aslam::backend::EuclideanExpression & rx_b,
			const aslam::backend::EuclideanExpression & ry_b,
			const aslam::backend::EuclideanExpression & rz_b,
			const aslam::backend::EuclideanExpression & bias,
			const aslam::backend::EuclideanExpression & g_w);
	virtual ~AccelerometerErrorEccentric();

private:
	aslam::backend::MatrixBasic _Ix, _Iy, _Iz;
};

}  // namespace kalibr

#endif /* KALIBR_IMU_CAM_ACCELEROMETER_ERROR_HPP */
