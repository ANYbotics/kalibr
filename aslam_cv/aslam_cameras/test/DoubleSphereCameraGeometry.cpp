// Bring in gtest
#include <gtest/gtest.h>
#include <sm_eigen/gtest.hpp>
#include <aslam_cameras/cameras.hpp>
#include <aslam_cameras/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testDoubleSphereCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DoubleSphereCameraGeometry> harness(0.1);

  SCOPED_TRACE("double sphere camera");
  harness.testAll();

}
