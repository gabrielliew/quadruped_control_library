#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP

#include <Eigen/Core>

/**
 * @brief Creating a Leg Controller Class. Assumes the legs have same axis for
 * all joints
 *
 */
class LegController {
public:
  LegController(int side, double hip, double thigh, double knee,
                Eigen::Vector3d kP, Eigen::Vector3d kD);
  void updatePosition(Eigen::Vector3d position) { position_ = position; }
  void updateVelocity(Eigen::Vector3d velocity) { velocity_ = velocity; }
  Eigen::Vector3d computeFootPosition();
  Eigen::Matrix3d computelegJacobian();
  Eigen::Vector3d getTorque(Eigen::Vector3d desiredPosition,
                            Eigen::Vector3d desiredVelocity,
                            Eigen::Vector3d desiredForce);

private:
  /**
   * @brief Positive for left side, and Negative for right side
   *
   */
  int side_;
  double hip_;
  double thigh_;
  double knee_;

  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Matrix3d kP_;
  Eigen::Matrix3d kD_;
};

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_LEGCONTROLLER_HPP */
