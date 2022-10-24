#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP

#include <Eigen/Core>

namespace QuadrupedLeg {
/**
 * @brief Creating a Leg Controller Class. Assumes the legs have same axis for
 * all joints
 *
 */
struct legCartesianCommand {
  Eigen::Vector3d feedForwardForce = Eigen::Vector3d::Zero();
  Eigen::Vector3d pDesired = Eigen::Vector3d::Zero();
  Eigen::Vector3d vDesired = Eigen::Vector3d::Zero();
  Eigen::Matrix3d kP = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d kd = Eigen::Matrix3d::Zero();
};

struct legJointCommand {
  Eigen::Vector3d feedForwardTorque = Eigen::Vector3d::Zero();
  Eigen::Vector3d qDesired = Eigen::Vector3d::Zero();
  Eigen::Vector3d qdDesired = Eigen::Vector3d::Zero();
  Eigen::Matrix3d kP = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d kd = Eigen::Matrix3d::Zero();
};

class LegController {
public:
  LegController(int side, double hip, double thigh, double knee,
                Eigen::Vector3d hipLocation);

  /**
   * @brief For updating the robot leg position during simulation
   *
   * @param q
   */
  void updateQ(Eigen::Vector3d q) { q_ = q; }

  /**
   * @brief For updating the robot leg velocity during simulation
   *
   * @param qd
   */
  void updateQD(Eigen::Vector3d qd) { qd_ = qd; }

  int getSide() { return side_; }

  Eigen::Vector3d computeFootPosition();

  Eigen::Vector3d computeFootPositionCOM() {
    return computeFootPosition() + hipLocation_;
  }

  Eigen::Matrix3d computelegJacobian();
  Eigen::Vector3d getTorque(legCartesianCommand desiredCommand);
  Eigen::Vector3d getTorque(legJointCommand desiredCommand);

private:
  /**
   * @brief Positive for left side, and Negative for right side
   *
   */
  int side_;
  double hip_;
  double thigh_;
  double knee_;
  Eigen::Vector3d hipLocation_;
  Eigen::Vector3d q_;
  Eigen::Vector3d qd_;
};
} // namespace QuadrupedLeg
#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP */
