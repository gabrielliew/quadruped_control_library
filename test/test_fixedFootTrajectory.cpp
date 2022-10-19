#include <gtest/gtest.h>

#include "quadruped_control_library/fixedFootTrajectory.hpp"

#define ABS_ERROR 0.001

using namespace FixedFootTrajectory;

TEST(FixedFootTrajectory, computeSwingTrajectoryBezier)
{
    FootTrajectory fixedFootTrajectory;
    Eigen::Vector3d _p0(1, 1, 1);
    fixedFootTrajectory.setInitialPosition(_p0);
    Eigen::Vector3d pf(2, 2, 1.2);
    fixedFootTrajectory.setFinalPosition(pf);
    fixedFootTrajectory.setHeight(1);

    double phasePerSecond = 2.; // 0.5 second swing.
    double t0 = 0.27;
    double dt = 0.001;
    double t1 = t0 + dt;
    double ph0 = t0 * phasePerSecond;
    double ph1 = t1 * phasePerSecond;

    fixedFootTrajectory.computeSwingTrajectoryBezier(ph0, 0.5);
    auto p0 = fixedFootTrajectory.getPosition();
    auto v0 = fixedFootTrajectory.getVelocity();
    auto a0 = fixedFootTrajectory.getAcceleration();

    fixedFootTrajectory.computeSwingTrajectoryBezier(ph1, 0.5);
    auto p1 = fixedFootTrajectory.getPosition();
    auto v1 = fixedFootTrajectory.getVelocity();
    auto a1 = fixedFootTrajectory.getAcceleration();

    Eigen::Vector3d vdiff = (p1 - p0) / dt;
    Eigen::Vector3d vref = (v0 + v1) / 2;
    Eigen::Vector3d adiff = (v1 - v0) / dt;
    Eigen::Vector3d aref = (a0 + a1) / 2;

    for (int i = 0; i < 3; i++)
    {
        EXPECT_NEAR(adiff[i], aref[i], ABS_ERROR);
        EXPECT_NEAR(vdiff[i], vref[i], ABS_ERROR);
    }
}
