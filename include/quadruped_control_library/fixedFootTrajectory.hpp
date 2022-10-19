#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTTRAJECTORY_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTTRAJECTORY_HPP

#include <Eigen/Core>

namespace FixedFootTrajectory
{
    class FootTrajectory
    {
    public:
        FootTrajectory()
        {
            p0_.setZero();
            pf_.setZero();
            p_.setZero();
            v_.setZero();
            a_.setZero();
            height_ = 0;
        }

        void setInitialPosition(Eigen::Vector3d p0)
        {
            p0_ = p0;
        }

        void setFinalPosition(Eigen::Vector3d pf)
        {
            pf_ = pf;
        }

        void setHeight(double h)
        {
            height_ = h;
        }

        Eigen::Vector3d getPosition()
        {
            return p_;
        }

        Eigen::Vector3d getVelocity()
        {
            return v_;
        }

        Eigen::Vector3d getAcceleration()
        {
            return a_;
        }

        void computeSwingTrajectoryBezier(double phase, double swingTime);
        double cubicBezier(double y0, double yf, double x);
        double cubicBezierFirstDerivative(double y0, double yf, double x);
        double cubicBezierSecondDerivative(double y0, double yf, double x);

    private:
        Eigen::Vector3d p0_, pf_, p_, v_, a_;
        double height_;
    };
}

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_FIXEDFOOTTRAJECTORY_HPP */
