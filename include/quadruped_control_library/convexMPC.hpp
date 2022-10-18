#ifndef INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP
#define INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <vector>

/**
 * @brief Contains the required components of the Model Predictive Control (MPC)
 *
 */
struct MPCdata
{
    /**
     * @brief Rotation angle of the robot in world frame (radians)
     *
     */
    Eigen::Vector3d rotation;
    /**
     * @brief Position of the robot in world frame (m)
     *
     */
    Eigen::Vector3d position;
    /**
     * @brief Rotational velocity of the robot in world frame (radians/second)
     *
     */
    Eigen::Vector3d rotation_velocity;
    /**
     * @brief Linear velocity of the robot in world frame (m/second)
     *
     */
    Eigen::Vector3d linear_velocity;
};

class ConvexMPC
{
public:
    /**
     * @brief Construct a new ConvexMPC object
     *
     * @param horizon Number of horizons within the MPC
     * @param dtMPC Time between horizons within the MPC
     * @param gravity World gravity, usually -9.8 (m^2/s)
     * @param numLegs Number of legs of the robot
     */
    ConvexMPC(uint horizon, double dtMPC, double gravity, uint numLegs);

    /**
     * @brief Updates the body inertia of the robot in the case of added payload that significantly modifies robot inertia
     *
     * @param IBody The simplified body inertia of the robot with size <3,3>
     */
    void updateBodyInertia(Eigen::Matrix<double, 3, 3> IBody)
    {
        IBody_ = IBody;
    }

    /**
     * @brief Updates the body mass of the robot
     *
     * @param mass The mass of the robot in kilograms
     */
    void updateBodyMass(double mass)
    {
        mass_ = mass;
    }

    /**
     * @brief Updates the friction coefficient between foot and the ground
     *
     * @param mu Friction coefficient which is 0<mu<1
     */
    void updateFrictionCoefficient(double mu)
    {
        mu_ = mu;
    }

    void updateMaxForce(double maxForce)
    {
        maxForce_ = maxForce;
    }
    /**
     * @brief create state weight matrice
     *
     * @param stateWeights
     */
    void updateStateWeights(Eigen::Matrix<double, 13, 1> stateWeights);
    void updateForceWeights(double forceWeights);
    std::vector<Eigen::Vector3d> solveMPC(const MPCdata &mpcData, const MPCdata &desiredState, const std::vector<bool> &gaitTable, const Eigen::Matrix<double, 3, -1> &rFeet);
    Eigen::Matrix<double, -1, 1> getTrajectory(const MPCdata &desiredState);
    std::tuple<std::vector<bool>, uint> getGaitData(const std::vector<bool> &gaitTable);
    Eigen::Matrix<double, -1, 1> getLowerBoundary(uint numLegsActive);
    Eigen::Matrix<double, -1, 1> getUpperBoundary(uint numLegsActive, double maxForce);
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> getConstraintMatrix(uint numLegsActive, double mu);
    Eigen::Matrix<double, 13, 13> getAdt(const Eigen::Matrix<double, 3, 3> &rYaw);
    Eigen::Matrix<double, 13, -1> getBdt(const Eigen::Matrix<double, 3, 3> &rYaw, const Eigen::Matrix<double, 3, -1> &rFeet);
    std::tuple<Eigen::Matrix<double, -1, 13>, Eigen::Matrix<double, -1, -1>> getAqp_Bqp(const Eigen::Matrix<double, 13, 13> &Adt, Eigen::Matrix<double, 13, -1> &Bdt);
    std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>> getH_g(const Eigen::Matrix<double, 13, 1> &x0, const Eigen::Matrix<double, -1, 1> &xD, const Eigen::Matrix<double, -1, 13> &Aqp, const Eigen::Matrix<double, -1, -1> &Bqp);
    std::tuple<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>, Eigen::Matrix<double, -1, 1>> getHRed_gRed(const Eigen::Matrix<double, -1, -1> &H, const Eigen::Matrix<double, -1, 1> &g, std::vector<bool> reductionVector, uint numLegsActive);

private:
    uint horizon_;
    double dtMPC_;
    double gravity_;
    double mass_;
    uint numLegs_;
    double mu_;
    double maxForce_;
    Eigen::Matrix<double, 3, 3> IBody_;
    Eigen::Matrix<double, -1, -1> stateWeights_;
    Eigen::Matrix<double, -1, -1> forceWeights_;
};

#endif /* INCLUDE_QUADRUPED_CONTROL_LIBRARY_CONVEXMPC_HPP */
