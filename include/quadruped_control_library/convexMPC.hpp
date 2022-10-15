#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <vector>

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
    void updateFrictionCoefficient(double mu)
    {
        mu_ = mu;
    }
    void updateStateWeights(Eigen::Matrix<double, 13, 1> stateWeights);
    void updateForceWeights(double forceWeights);
    std::vector<double> solveMPC(const MPCdata &mpcData, const MPCdata &desiredState, const std::vector<bool> &gaitTable, const Eigen::Matrix<double, 3, -1> &rFeet);
    Eigen::Matrix<double, -1, 1> getTrajectory(const MPCdata &desiredState);
    std::tuple<std::vector<bool>, uint> getGaitData(const std::vector<bool> &gaitTable);
    Eigen::Matrix<double, -1, 1> getLowerBoundary(uint numLegsActive);
    Eigen::Matrix<double, -1, 1> getUpperBoundary(uint numLegsActive, double maxForce);
    Eigen::Matrix<double, -1, -1> getConstraintMatrix(uint numLegsActive, double mu);
    Eigen::Matrix<double, 13, 13> getAdt(const Eigen::Matrix<double, 3, 3> &rYaw);
    Eigen::Matrix<double, 13, -1> getBdt(const Eigen::Matrix<double, 3, 3> &rYaw, const Eigen::Matrix<double, 3, -1> &rFeet);
    std::tuple<Eigen::Matrix<double, -1, 13>, Eigen::Matrix<double, -1, -1>> getAqp_Bqp(const Eigen::Matrix<double, 13, 13> &Adt, Eigen::Matrix<double, 13, -1> &Bdt);
    std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>> getH_g(const Eigen::Matrix<double, 13, 1> &x0, const Eigen::Matrix<double, -1, 1> &xD, const Eigen::Matrix<double, -1, 13> &Aqp, const Eigen::Matrix<double, -1, -1> &Bqp);
    std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>> getHRed_gRed(const Eigen::Matrix<double, -1, -1> &H, const Eigen::Matrix<double, -1, 1> &g, std::vector<bool> reductionVector, uint numLegsActive);
    // Eigen::Matrix<double, -1, -1> getReduced

private:
    uint horizon_;
    double dtMPC_;
    double gravity_;
    double mass_;
    uint numLegs_;
    double mu_;
    Eigen::Matrix<double, 3, 3> IBody_;
    Eigen::Matrix<double, -1, -1> stateWeights_;
    Eigen::Matrix<double, -1, -1> forceWeights_;
};

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP */
