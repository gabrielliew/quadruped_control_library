#include "quadruped_control_library/convexMPC.hpp"

ConvexMPC::ConvexMPC(uint horizon, double dtMPC, double gravity, uint numLegs)
{
    horizon_ = horizon;
    dtMPC_ = dtMPC;
    gravity_ = gravity;
    numLegs_ = numLegs;
}

Eigen::Matrix<double, -1, 1> ConvexMPC::solveMPC(MPCdata mpcData, MPCdata desiredState)
{
    Eigen::Matrix<double, 13, 1> x0;
    x0 << mpcData.rotation[0], mpcData.rotation[1], mpcData.rotation[2],
        mpcData.position[0], mpcData.position[1], mpcData.position[2],
        mpcData.rotation_velocity[0], mpcData.rotation_velocity[1], mpcData.rotation_velocity[2],
        mpcData.linear_velocity[0], mpcData.linear_velocity[1], mpcData.linear_velocity[2],
        gravity_;

    double yc = std::cos(mpcData.rotation[2]);
    double ys = std::sin(mpcData.rotation[2]);
    RYaw::Matrix<double, 3, 3> RYaw;
    RYaw << yc, -ys, 0,
        ys, yc, 0,
        0, 0, 1;

    auto Adt = getAdt();
    auto Bdt = getBdt(RYaw);
    auto [Aqp, Bqp] = getAqp_Bqp(Adt, Bdt);
    auto xD = getTrajectory(desiredState);
    auto [qH, qG] = getqH_qG(x0, xD);
    Eigen::Matrix<double, -1, 1> test;
    return test;
}

Eigen::Matrix<double, -1, 1> ConvexMPC::getTrajectory(const MPCdata &desiredState)
{
    Eigen::Matrix<double, -1, 1> trajectory;
    trajectory.resize(13 * horizon_);
    Eigen::Matrix<double, 13, 1> trajectoryInitial;
    // set roll,pitch,vertical velocity and gravity to zero (useless components)
    trajectoryInitial << desiredState.rotation[0], desiredState.rotation[1], desiredState.rotation[2],
        desiredState.position[0], desiredState.position[1], desiredState.position[2],
        0.0, 0.0, desiredState.rotation_velocity[2],
        desiredState.linear_velocity[0], desiredState.linear_velocity[1], 0.0,
        0;
    for (int i = 0; i < horizon_; i++)
    {
        trajectory.block(i * 13, 0, 13, 1) = trajectoryInitial;
        trajectory(i * 13 + 2, 0) = desiredState.rotation[2] + i * dtMPC_ * desiredState.rotation_velocity[2];
        trajectory(i * 13 + 3, 0) = desiredState.position[0] + i * dtMPC_ * desiredState.linear_velocity[0];
        trajectory(i * 13 + 4, 0) = desiredState.position[1] + i * dtMPC_ * desiredState.linear_velocity[1];
    }
    return trajectory;
}

Eigen::Matrix<double, -1, 1> ConvexMPC::getLowerLimit(uint numLegsActive)
{
    return Eigen::Matrix<double, -1, 1>::Constant(numLegsActive * 5, 0.0);
}

Eigen::Matrix<double, -1, 1> ConvexMPC::getUpperLimit(uint numLegsActive, double maxForce)
{
    Eigen::Matrix<double, -1, 1> upperLimit(numLegsActive * 5, 1);
    for (int i = 0; i < numLegsActive; i++)
    {
        upperLimit(i * 5 + 0, 0) = std::numeric_limits<double>::max();
        upperLimit(i * 5 + 1, 0) = std::numeric_limits<double>::max();
        upperLimit(i * 5 + 2, 0) = std::numeric_limits<double>::max();
        upperLimit(i * 5 + 3, 0) = std::numeric_limits<double>::max();
        upperLimit(i * 5 + 4, 0) = maxForce;
    }
    return upperLimit;
}

Eigen::Matrix<double, -1, -1> ConvexMPC::getConstraintMatrix(uint numLegsActive, double mu)
{
    Eigen::Matrix<double, 5, 3> constraintBlock;
    double muInv = 1.0 / mu;
    constraintBlock << muInv, 0.0, 1.0,
        -muInv, 0.0, 1.0,
        0.0, muInv, 1.0,
        0.0, -muInv, 1.0,
        0.0, 0.0, 1.0;
    Eigen::Matrix<double, -1, -1> constraintMatrix(numLegsActive * 5, numLegsActive * 3);
    constraintMatrix.setZero();
    for (int i = 0; i < numLegsActive; i++)
    {
        constraintMatrix.block(i * 5, i * 3, 5, 3) = constraintBlock;
    }
    return constraintMatrix;
}

Eigen::Matrix<double, 13, 13> ConvexMPC::getAdt()
{
    Eigen::Matrix<double, 13, 13> Adt;
    Adt.setZero();
    Adt.setIdentity();
    Adt.block(0, 6, 3, 3) = RYaw_.transpose() * dtMPC_;
    Adt.block(3, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dtMPC_;
    Adt(11, 12) = dtMPC_;
    return Adt;
}

Eigen::Matrix<double, 13, -1> ConvexMPC::getBdt(const Eigen::Matrix<double, 3, 3> &rYaw)
{
    Eigen::Matrix<double, 13, -1> Bdt;
    Bdt.resize(13, 3 * numLegs_);
    Bdt.setZero();
    Eigen::Matrix<double, 3, 3> IWorld_inv;
    IWorld_inv = rYaw * IBody_ * rYaw.transpose();
    IWorld_inv = IWorld_inv.inverse();
    for (int i = 0; i < numLegs_; i++)
    {
        Eigen::Matrix<double, 3, 3> crossMat;
        crossMat << 0.0, -rFeet_(2, i), rFeet_(1, i),
            rFeet_(2, i), 0.0, -rFeet_(0, i),
            -rFeet_(1, i), rFeet_(0, i), 0.0;
        Bdt.block(6, i * 3, 3, 3) = IWorld_inv * crossMat;
        Bdt.block(9, i * 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() / mass_;
    }
    return Bdt;
}

std::tuple<Eigen::Matrix<double, -1, 13>, Eigen::Matrix<double, -1, -1>> ConvexMPC::getAqp_Bqp(const Eigen::Matrix<double, 13, 13> &Adt, Eigen::Matrix<double, 13, -1> &Bdt)
{
    Eigen::Matrix<double, 13, 13> powerMats[20];
    Eigen::Matrix<double, -1, 13> Aqp;
    Eigen::Matrix<double, -1, -1> Bqp;
    Aqp.resize(13 * horizon_, 13);
    Aqp.setZero();
    Bqp.resize(13 * horizon_, numLegs_ * 3 * horizon_);
    Bqp.setZero();
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon_ + 1; i++)
    {
        powerMats[i] = Adt * powerMats[i - 1];
    }
    for (int r = 0; r < horizon_; r++)
    {
        Aqp.block(13 * r, 0, 13, 13) = powerMats[r + 1];
        for (int c = 0; c < horizon_; c++)
        {
            if (r >= c)
            {
                int a_num = r - c;
                Bqp.block(13 * r, numLegs_ * 3 * c, 13, numLegs_ * 3) = powerMats[a_num] * Bdt;
            }
        }
    }
    return {Aqp, Bqp};
}

std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>> ConvexMPC::getqH_qG(const Eigen::Matrix<double, 13, 1> &x0, const Eigen::Matrix<double, -1, 1> &xD)
{

    Eigen::Matrix<double, -1, -1> qH;
    Eigen::Matrix<double, -1, 1> qG;
    qH.resize(3 * numLegs_ * horizon_, 3 * numLegs_ * horizon_);
    qH = 2 * Bqp.transpose() * stateWeights_ * Bqp + forceWeights_;
    qG = 2 * Bqp.transpose() * stateWeights_ * (Aqp * x0 - xD);
    return {qH, qG};
}