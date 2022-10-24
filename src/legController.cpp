#include "quadruped_control_library/legController.hpp"
#include <cmath>

namespace QuadrupedLeg {
LegController::LegController(int side, double hip, double thigh, double knee,
                             Eigen::Vector3d hipLocation) {
  hip_ = hip;
  thigh_ = thigh;
  knee_ = knee;
  side_ = side;
  hipLocation_ = hipLocation;
}

Eigen::Vector3d LegController::computeFootPosition() {
  double sin0 = std::sin(q_[0]);
  double sin1 = std::sin(q_[1]);
  double sin12 = std::sin(q_[1] + q_[2]);
  double cos0 = std::cos(q_[0]);
  double cos1 = std::cos(q_[1]);
  double cos12 = std::cos(q_[1] + q_[2]);

  double x = -thigh_ * sin1 - knee_ * sin12;
  double y = hip_ * side_ * cos0 + sin0 * (thigh_ * cos1 + knee_ * cos12);
  double z = hip_ * side_ * sin0 - cos0 * (thigh_ * cos1 + knee_ * cos12);

  return Eigen::Vector3d(x, y, z);
}

Eigen::Matrix3d LegController::computelegJacobian() {
  Eigen::Matrix3d jacobian;
  double sin0 = std::sin(q_[0]);
  double sin1 = std::sin(q_[1]);
  double sin12 = std::sin(q_[1] + q_[2]);
  double cos0 = std::cos(q_[0]);
  double cos1 = std::cos(q_[1]);
  double cos12 = std::cos(q_[1] + q_[2]);
  jacobian(0, 0) = 0;
  jacobian(0, 1) = -thigh_ * cos1 - knee_ * cos12;
  jacobian(0, 2) = -knee_ * cos12;
  jacobian(1, 0) = cos0 * (thigh_ * cos1 + knee_ * cos12) - hip_ * side_ * sin0;
  jacobian(1, 1) = -sin0 * (thigh_ * sin1 + knee_ * sin12);
  jacobian(1, 2) = -sin0 * (knee_ * sin12);
  jacobian(2, 0) = hip_ * side_ * cos0 + sin0 * (thigh_ * cos1 + knee_ * cos12);
  jacobian(2, 1) = cos0 * (thigh_ * sin1 + knee_ * sin12);
  jacobian(2, 2) = cos0 * (knee_ * sin12);
  return jacobian;
}

Eigen::Vector3d LegController::getTorque(legCartesianCommand desiredCommand) {
  Eigen::Vector3d finalTorque = Eigen::Vector3d(0, 0, 0);
  auto position = computeFootPosition();
  auto velocity = computelegJacobian()*qd_;
  finalTorque += desiredCommand.kP * (desiredCommand.pDesired - position);
  finalTorque += desiredCommand.kd * (desiredCommand.vDesired - velocity);
  finalTorque += computelegJacobian().transpose() * desiredCommand.feedForwardForce;
  return finalTorque;
}

Eigen::Vector3d LegController::getTorque(legJointCommand desiredCommand) {
  Eigen::Vector3d finalTorque = Eigen::Vector3d(0, 0, 0);
  finalTorque += desiredCommand.kP * (desiredCommand.qDesired - q_);
  finalTorque += desiredCommand.kd * (desiredCommand.qdDesired - qd_);
  finalTorque += desiredCommand.feedForwardTorque;
  return finalTorque;
}
} // namespace QuadrupedLeg