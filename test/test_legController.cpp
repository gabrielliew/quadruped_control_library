#include <gtest/gtest.h>

#include "quadruped_control_library/legController.hpp"

#define ABS_ERROR 0.001
using namespace QuadrupedLeg;
LegController leftLeg(1, 0.062, 0.209, 0.195, Eigen::Vector3d(0, 0, 0));
LegController rightLeg(-1, 0.062, 0.209, 0.195, Eigen::Vector3d(0, 0, 0));

TEST(LegController, computeFootPosition) {
  Eigen::Vector3d leftTest, rightTest, position;

  position = Eigen::Vector3d(0, 0, 0);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.062, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], -0.404, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.062, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], -0.404, ABS_ERROR);

  position = Eigen::Vector3d(1.570796, 0, 0);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.404, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], 0.062, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], 0.404, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], -0.062, ABS_ERROR);

  position = Eigen::Vector3d(-1.570796, 0, 0);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], -0.404, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], -0.062, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], 0, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.404, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], 0.062, ABS_ERROR);

  position = Eigen::Vector3d(0, 1.570796, 0);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], -0.404, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.062, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], 0, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], -0.404, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.062, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], 0, ABS_ERROR);

  position = Eigen::Vector3d(0, -1.570796, 0);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], 0.404, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.062, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], 0, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], 0.404, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.062, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], 0, ABS_ERROR);

  position = Eigen::Vector3d(0, 0, -1.570796);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], 0.195, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.062, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], -0.209, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], 0.195, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.062, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], -0.209, ABS_ERROR);

  position = Eigen::Vector3d(0, 0, 1.570796);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], -0.195, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.062, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], -0.209, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], -0.195, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], -0.062, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], -0.209, ABS_ERROR);

  position = Eigen::Vector3d(0.785398, 0.785398, 0.785398);
  leftLeg.updateQ(position);
  rightLeg.updateQ(position);
  leftTest = leftLeg.computeFootPosition();
  EXPECT_NEAR(leftTest[0], -0.342785, ABS_ERROR);
  EXPECT_NEAR(leftTest[1], 0.148341, ABS_ERROR);
  EXPECT_NEAR(leftTest[2], -0.060659, ABS_ERROR);
  rightTest = rightLeg.computeFootPosition();
  EXPECT_NEAR(rightTest[0], -0.342785, ABS_ERROR);
  EXPECT_NEAR(rightTest[1], 0.0606594, ABS_ERROR);
  EXPECT_NEAR(rightTest[2], -0.148341, ABS_ERROR);
}

/**
 *   x_new = x_old + small_x_change
 *         = x_old + Jacobian * small_x_change
 */
TEST(LegController, computelegJacobian) {
  Eigen::Vector3d x0(0.3906, 0.6918, 0.3814);
  Eigen::Vector3d dx(-0.00143, 0.00222, -0.00199);
  Eigen::Vector3d x1;
  x1 = x0 + dx;
  leftLeg.updateQ(x0);
  auto p0 = leftLeg.computeFootPosition();
  auto J0 = leftLeg.computelegJacobian();
  auto p1 = p0 + J0 * dx;
  leftLeg.updateQ(x1);
  auto example_p1 = leftLeg.computeFootPosition();
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(p1[i], example_p1[i], ABS_ERROR);
  }
}