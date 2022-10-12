#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP

#include <Eigen/Core>
#include <vector>

class FixedGait
{
public:
    /**
     * @brief Construct a new Gait object
     *
     * @param numLegs Number of legs
     * @param numSegments Number of segments in one gait cycle
     * @param gaitDuration How long the one gait cycle is in seconds
     * @param contactDuration How long does one contact period is in seconds (contact Duration should be in multiples of gaitDurationNs/numSegments)
     * @param offset offset of contact duration from the start of the cycle (default is 0, max is (gaitDurationNs-contactDurationNs)/numSegments)
     */
    FixedGait(uint numLegs, uint numSegments, double gaitDuration, double contactDuration, std::vector<int> offset);

    /**
     * @brief Get the Swing Progression object
     *
     * @param timeFromStart Takes the time from start to calculate the swing progression of each leg
     * @return std::array<double, numLegs>
     */
    std::vector<double> getSwingProgression(uint64_t timeFromStart);
    std::vector<double> getContactProgression(uint64_t timeFromStart);
    Eigen::Matrix<int, -1, -1> getGaitTable(uint64_t timeFromStart);

private:
    uint numSegments_;
    uint numLegs_;
    uint64_t gaitDurationNs_;
    uint64_t contactDurationNs_;
    uint64_t swingDurationNs_;
    uint64_t singleSegmentNs_;
    std::vector<uint64_t> offset_;
};

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDGAIT_HPP */
