#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTPLANNER_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTPLANNER_HPP

#include <Eigen/Core>
#include <vector>

namespace FixedFootPlanner {

struct FootPlannerData {
  Eigen::Vector3d worldPosition;
  Eigen::Vector3d worldVelocity;
  Eigen::Matrix3d rBody;
  Eigen::Vector3d desiredVelocity;
  std::vector<Eigen::Vector3d> legPositions;
  std::vector<bool> firstSwing;
};

class FootPlanner {
public:
  FootPlanner(){};
  FootPlanner(uint numLegs, std::vector<Eigen::Vector3d> hipLocation,
              double swingDuration, double contactDuration, double dt);

  std::vector<Eigen::Vector3d> runPlanner(FootPlannerData footPlannerData);

private:
  uint numLegs_;
  std::vector<Eigen::Vector3d> hipLocation_;
  double contactDuration_;
  double swingDuration_;
  double dt_;
  std::vector<double> swingTimeRemaining_;
};
} // namespace FixedFootPlanner

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTPLANNER_HPP */
