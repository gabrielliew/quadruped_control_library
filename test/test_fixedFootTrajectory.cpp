#include <gtest/gtest.h>

#include "quadruped_control_library/fixedFootTrajectory.hpp"

#define ABS_ERROR 0.00001


TEST(FixedFootTrajectory, getSwingProgression)
{
    EXPECT_NEAR(1, 1, ABS_ERROR);
}
