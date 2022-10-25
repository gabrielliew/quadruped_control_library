#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP

#include <Eigen/Core>
#include <vector>

namespace FixedGait {
class Gait {
public:
  Gait(){};
  /**
   * @brief Construct a new Gait object
   *
   * @param numLegs Number of legs
   * @param numSegments Number of segments in one gait cycle
   * @param gaitDuration How long the one gait cycle is in secondu
   * @param contactDuration How long does one contact period is in seconds
   * (contact Duration should be in multiples of gaitDurationNs/numSegments)
   * @param offset offset of contact duration from the start of the cycle
   * (default is 0, max is (gaitDurationNs-contactDurationNs)/numSegments)
   */
  Gait(uint numLegs, uint numSegments, double gaitDuration,
       double contactDuration, std::vector<int> offset);

  /**
   * @brief Get the Swing Progression of robot legs
   *
   * @param timeFromStart Takes the time from start to calculate the swing
   * progression of each leg
   * @return std::vector<double> The array containing swing progression of leg
   * in [0.0,1.0] range
   */
  std::vector<double> getSwingProgression(uint64_t timeFromStart);

  /**
   * @brief Get the Contact Progression of robot legs
   *
   * @param timeFromStart Takes the time from start to calculate the contact
   * progression of each leg
   * @return std::vector<double> The array containing contact progression of leg
   * in [0.0,1.0] range
   */
  std::vector<double> getContactProgression(uint64_t timeFromStart);

  /**
   * @brief Get the Gait Table of the robot for Model Predictive Control (MPC)
   * calculation
   *
   * @param timeFromStart Takes the time from start to calculate the contact
   * progression of each leg
   * @return std::vector<bool> Returns a vector with size number of legs *
   * number of gait segments, the value of 1 indicates contact and 0 indicates
   * swing
   */
  std::vector<bool> getGaitTable(uint64_t timeFromStart);

  double getSwingDuration() { return swingDuration_; }
  double getContactDuration() { return contactDuration_; }

private:
  /**
   * @brief Number of segments within a gait cycle
   *
   */
  uint numSegments_;

  /**
   * @brief Number of legs of the robot
   *
   */
  uint numLegs_;

  /**
   * @brief The duration of one gait cycle (nanoseconds)
   * total gait duration = contact phase duration + swing phase duration
   *
   */
  uint64_t gaitDurationNs_;

  /**
   * @brief The duration of contact phase in one gait cycle (nanoseconds)
   *
   */
  uint64_t contactDurationNs_;

  /**
   * @brief The duration of swing phase in one gait cycle (nanoseconds)
   *
   */
  uint64_t swingDurationNs_;

  /**
   * @brief The duration of a single segment of gait cycle, all duration types
   * should be multiples of this value
   *
   */
  uint64_t singleSegmentNs_;

  /**
   * @brief The offset of each leg in the gait cycle in segments, 0 indicates
   * the leg starts contact at start of gait cycle
   *
   */
  std::vector<uint64_t> offset_;

  double swingDuration_;
  double contactDuration_;
};
} // namespace FixedGait
#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP */
