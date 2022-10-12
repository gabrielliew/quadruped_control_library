#include "quadruped_control_library/fixedFootTrajectory.hpp"

#include <iostream>

void FixedFootTrajectory::computeSwingTrajectoryBezier(double swingProgression, double swingTime)
{
    for (int i = 0; i < 2; i++)
    {
        p_[i] = cubicBezier(p0_[i], pf_[i], swingProgression);
        v_[i] = cubicBezierFirstDerivative(p0_[i], pf_[i], swingProgression) / swingTime;
        a_[i] = cubicBezierSecondDerivative(p0_[i], pf_[i], swingProgression) / (swingTime * swingTime);
    }

    if (swingProgression < 0.5)
    {
        p_[2] = cubicBezier(p0_[2], p0_[2] + height_, swingProgression * 2);
        v_[2] = cubicBezierFirstDerivative(p0_[2], p0_[2] + height_, swingProgression * 2) * 2 / swingTime;
        a_[2] = cubicBezierSecondDerivative(p0_[2], p0_[2] + height_, swingProgression * 2) * 4 / (swingTime * swingTime);
    }
    else
    {
        p_[2] = cubicBezier(p0_[2] + height_, pf_[2], swingProgression * 2 - 1);
        v_[2] = cubicBezierFirstDerivative(p0_[2] + height_, pf_[2], swingProgression * 2 - 1) * 2 / swingTime;
        a_[2] = cubicBezierSecondDerivative(p0_[2] + height_, pf_[2], swingProgression * 2 - 1) * 4 / (swingTime * swingTime);
    }
}

double FixedFootTrajectory::cubicBezier(double y0, double yf, double x)
{
    double yDiff = yf - y0;
    double bezier = x * x * x + 3 * (x * x * (1 - x));
    return y0 + bezier * yDiff;
}

double FixedFootTrajectory::cubicBezierFirstDerivative(double y0, double yf, double x)
{
    double yDiff = yf - y0;
    double bezier = 6 * x * (1 - x);
    return bezier * yDiff;
}

double FixedFootTrajectory::cubicBezierSecondDerivative(double y0, double yf, double x)
{
    double yDiff = yf - y0;
    double bezier = 6 - 12 * x;
    return bezier * yDiff;
}
