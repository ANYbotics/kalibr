// Bring in gtest
#include <gtest/gtest.h>
#include <sm_eigen/gtest.hpp>
#include <aslam_cameras/cameras/DepthCameraGeometry.hpp>
#include <aslam_cameras/cameras/test/CameraGeometryTestHarness.hpp>

TEST(AslamCamerasTestSuite, testDepthCameraGeometry)
{
  using namespace aslam::cameras;
  CameraGeometryTestHarness<DepthCameraGeometry> harness(1e-2);
  SCOPED_TRACE("");
  harness.testAll();

}
