#include "quadruped_control_library/fixedFootPlanner.hpp"
#include <algorithm>

namespace FixedFootPlanner {
FootPlanner::FootPlanner(uint numLegs, std::vector<Eigen::Vector3d> hipLocation,
                         double swingDuration, double contactDuration,
                         double dt) {
  numLegs_ = numLegs;
  hipLocation_ = hipLocation;
  swingDuration_ = swingDuration;
  contactDuration_ = contactDuration;
  dt_ = dt;
  swingTimeRemaining_.insert(swingTimeRemaining_.begin(), numLegs_, 0);
}

std::vector<Eigen::Vector3d>
FootPlanner::runPlanner(FootPlannerData footPlannerData) {
  std::vector<Eigen::Vector3d> pFoot(numLegs_);
  for (int i = 0; i < numLegs_; i++) {
    pFoot[i] = footPlannerData.worldPosition +
               footPlannerData.rBody.transpose() *
                   (hipLocation_[i] + footPlannerData.legPositions[i]);
  }

  for (int l = 0; l < 4; l++)
    swingTimeRemaining_[l] = swingDuration_;

  double side_sign[4] = {-1, 1, -1, 1};
  //   double interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //   double interleave_gain = -0.2;
  //   double v_abs = std::fabs(v_des_robot[0]);
  std::vector<Eigen::Vector3d> Pf(4);
  for (int i = 0; i < numLegs_; i++) {

    if (footPlannerData.firstSwing[i]) {
      swingTimeRemaining_[i] = swingDuration_;
    } else {
      swingTimeRemaining_[i] -= dt_;
    }
    // if(firstSwing[i]) {
    // footSwingTrajectories[i].setHeight(.05);
    Eigen::Vector3d offset(0, side_sign[i] * .065, 0);

    Eigen::Vector3d pRobotFrame = hipLocation_[i]+offset;
    double yc =
        std::cos(-footPlannerData.desiredVelocity[2] * contactDuration_ / 2);
    double ys =
        std::sin(-footPlannerData.desiredVelocity[2] * contactDuration_ / 2);
    Eigen::Matrix<double, 3, 3> RYaw;
    RYaw << yc, -ys, 0, ys, yc, 0, 0, 0, 1;
    Eigen::Vector3d pYawCorrected = RYaw * pRobotFrame;

    Pf[i] = footPlannerData.worldPosition +
            footPlannerData.rBody.transpose() *
                (pYawCorrected +
                 footPlannerData.desiredVelocity * swingTimeRemaining_[i]);

    double maxPositionChange = 0.3;

    double pfx_rel = footPlannerData.worldVelocity[0] * 0.5 * contactDuration_ +
                     .03 * (footPlannerData.worldVelocity[0] -
                            footPlannerData.desiredVelocity[0]) +
                     (0.5 * footPlannerData.worldPosition[2] / 9.81) *
                         (footPlannerData.worldVelocity[1] *
                          footPlannerData.desiredVelocity[2]);

    double pfy_rel = footPlannerData.worldVelocity[1] * .5 * contactDuration_ +
                     .03 * (footPlannerData.worldVelocity[1] -
                            footPlannerData.desiredVelocity[1]) +
                     (0.5 * footPlannerData.worldPosition[2] / 9.81) *
                         (-footPlannerData.worldVelocity[0] *
                          footPlannerData.desiredVelocity[2]);
    pfx_rel =
        std::min(std::max(pfx_rel, -maxPositionChange), maxPositionChange);
    pfy_rel =
        std::min(std::max(pfy_rel, -maxPositionChange), maxPositionChange);
    Pf[i][0] += pfx_rel;
    Pf[i][1] += pfy_rel;
    Pf[i][2] = -0.003;
  }
  return Pf;
}
} // namespace FixedFootPlanner