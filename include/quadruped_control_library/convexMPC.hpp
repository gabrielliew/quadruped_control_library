#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

struct MPCdata
{
    Eigen::Vector3d rotation;
    Eigen::Vector3d position;
    Eigen::Vector3d rotation_velocity;
    Eigen::Vector3d linear_velocity;
};

class ConvexMPC
{
public:
    ConvexMPC(uint horizon, double dtMPC, double gravity, uint numLegs);
    void updateBodyInertia(Eigen::Matrix<double, 3, 3> IBody)
    {
        IBody_ = IBody;
    }
    void updateBodyMass(double mass)
    {
        mass_ = mass;
    }
    void updateStateWeights(Eigen::Matrix<double, 13, 1> stateWeights)
    {
        stateWeights_.resize(13 * horizon_, 13 * horizon_);
        stateWeights_.setIdentity();
        stateWeights_.diagonal() = stateWeights.replicate(horizon_, 1);
    }
    void updateForceWeights(double forceWeights)
    {
        forceWeights_.resize(3 * numLegs_ * horizon_, 3 * numLegs_ * horizon_);
        forceWeights_.setIdentity();
        forceWeights_ = forceWeights_ * forceWeights;
    }
    void updateFeetLocation(Eigen::Matrix<double, 3, -1> rFeet)
    {
        rFeet_ = rFeet;
    }
    Eigen::Matrix<double, -1, 1> solveMPC(MPCdata mpcData, MPCdata desiredState);
    Eigen::Matrix<double, -1, 1> getTrajectory(const MPCdata &desiredState);
    Eigen::Matrix<double, -1, 1> getLowerLimit(uint numLegsActive);
    Eigen::Matrix<double, -1, 1> getUpperLimit(uint numLegsActive, double maxForce);
    Eigen::Matrix<double, -1, -1> getConstraintMatrix(uint numLegsActive, double mu);
    Eigen::Matrix<double, 13, 13> getAdt();
    Eigen::Matrix<double, 13, -1> getBdt(const Eigen::Matrix<double, 3, 3> &rYaw);
    std::tuple<Eigen::Matrix<double, -1, 13>, Eigen::Matrix<double, -1, -1>> getAqp_Bqp(const Eigen::Matrix<double, 13, 13> &Adt, Eigen::Matrix<double, 13, -1> &Bdt);
    std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>> getqH_qG(const Eigen::Matrix<double, 13, 1> &x0, const Eigen::Matrix<double, -1, 1> &xD);
    

    // Eigen::Matrix<double, -1, -1> getReduced

private:
    uint horizon_;
    double dtMPC_;
    double gravity_;
    double mass_;
    uint numLegs_;
    Eigen::Matrix<double, 3, 3> IBody_;
    Eigen::Matrix<double, -1, -1> stateWeights_;
    Eigen::Matrix<double, -1, -1> forceWeights_;
};

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP */
