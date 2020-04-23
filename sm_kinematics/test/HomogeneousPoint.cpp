#include <sm_eigen/gtest.hpp>
#include <sm_kinematics/HomogeneousPoint.hpp>

#include "PointTestHarness.hpp"

TEST(HomogeneousPointTestSuite, testHpoints)
{
  sm::PointTestHarness<sm::kinematics::HomogeneousPoint> harness(1e-6);
  harness.testAll();
}
