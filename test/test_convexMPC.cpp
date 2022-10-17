#include <gtest/gtest.h>

#include "helper.hpp"
#include "save_load_eigen_csv.hpp"
#include "quadruped_control_library/convexMPC.hpp"
#include <iostream>

#define ABS_ERROR 0.0001

int horizon = 10;
int numLegs = 4;
int activeLegs = 4;
double maxForce = 120;
double mu = 0.4;
double mass = 9;
std::vector<bool> gaitTable = {0, 1, 1, 0,
                               0, 1, 1, 0,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               0, 1, 1, 0,
                               0, 1, 1, 0,
                               0, 1, 1, 0};

MPCdata desiredState;

ConvexMPC convexMPC(10, 0.026, -9.8, numLegs);

TEST(ConvexMPC, getLowerBoundary)
{
    auto lB = convexMPC.getLowerBoundary(20); // 0.5s passed
    for (int i = 0; i < 100; i++)
    {
        EXPECT_NEAR(lB(i, 0), 0, ABS_ERROR);
    }
}

TEST(ConvexMPC, getUpperBoundary)
{
    auto uB = convexMPC.getUpperBoundary(20, maxForce); // 0.5s passed
    auto example_uB = openData<double>(getTestDataCSV("uB"));
    for (int i = 0; i < 100; i++)
    {
        EXPECT_NEAR(uB(i, 0), example_uB(i, 0), ABS_ERROR);
    }
}

TEST(ConvexMPC, getConstraintMatrix)
{
    auto constraintMatrix = convexMPC.getConstraintMatrix(20, mu); // 0.5s passed
    auto A = openData<double>(getTestDataCSV("A"));
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 60; j++)
        {
            EXPECT_NEAR(A(i, j), constraintMatrix(i, j), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, getGaitData)
{
    auto [reductionVector, numLegsActive] = convexMPC.getGaitData(gaitTable);
    EXPECT_EQ(numLegsActive, 20);
    for (int i = 0; i < gaitTable.size(); i++)
    {
        EXPECT_EQ(reductionVector[i * 3], gaitTable[i]);
        EXPECT_EQ(reductionVector[i * 3 + 1], gaitTable[i]);
        EXPECT_EQ(reductionVector[i * 3 + 2], gaitTable[i]);
    }
}

TEST(ConvexMPC, getAdt)
{
    auto rYaw = openData<double>(getTestDataCSV("rYaw"));
    auto example_Adt = openData<double>(getTestDataCSV("Adt"));

    auto Adt = convexMPC.getAdt(rYaw);
    for (int i = 0; i < 13; i++)
    {
        for (int j = 0; j < 13; j++)
        {
            EXPECT_NEAR(example_Adt(i, j), Adt(i, j), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, getBdt)
{
    auto IBody = openData<double>(getTestDataCSV("IBody"));
    auto rYaw = openData<double>(getTestDataCSV("rYaw"));
    auto rFeet = openData<double>(getTestDataCSV("rFeet"));

    convexMPC.updateBodyInertia(IBody);
    convexMPC.updateBodyMass(mass);
    auto Bdt = convexMPC.getBdt(rYaw, rFeet);

    auto example_Bdt = openData<double>(getTestDataCSV("Bdt"));

    for (int i = 0; i < 13; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            EXPECT_NEAR(example_Bdt(i, j), Bdt(i, j), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, getAqp_Bqp)
{
    auto IBody = openData<double>(getTestDataCSV("IBody"));
    auto rYaw = openData<double>(getTestDataCSV("rYaw"));
    auto rFeet = openData<double>(getTestDataCSV("rFeet"));

    convexMPC.updateBodyInertia(IBody);
    convexMPC.updateBodyMass(mass);
    auto Bdt = convexMPC.getBdt(rYaw, rFeet);
    auto Adt = convexMPC.getAdt(rYaw);
    auto [Aqp, Bqp] = convexMPC.getAqp_Bqp(Adt, Bdt);

    auto example_Aqp = openData<double>(getTestDataCSV("Aqp"));
    auto example_Bqp = openData<double>(getTestDataCSV("Bqp"));
    for (int i = 0; i < 13 * horizon; i++)
    {
        for (int j = 0; j < 13; j++)
        {
            EXPECT_NEAR(Aqp(i, j), example_Aqp(i, j), ABS_ERROR);
        }
        for (int k = 0; k < numLegs * 3 * horizon; k++)
        {
            EXPECT_NEAR(Bqp(i, k), example_Bqp(i, k), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, getTrajectory)
{
    MPCdata desiredState;
    desiredState.rotation = Eigen::Vector3d(0.0042443, 0.0134921, 0.948284);
    desiredState.position = Eigen::Vector3d(-0.369856, 0.29033, 0.29);
    desiredState.rotation_velocity = Eigen::Vector3d(0, 0, 0.5);
    desiredState.linear_velocity = Eigen::Vector3d(-0.220608, 0.0354218, 0);
    auto xD = convexMPC.getTrajectory(desiredState);
    auto example_xD = openData<double>(getTestDataCSV("xD"));
    for (int i = 0; i < 13 * horizon; i++)
    {
        EXPECT_NEAR(xD(i, 0), example_xD(i, 0), ABS_ERROR);
    }
}

TEST(ConvexMPC, getH_g)
{
    MPCdata desiredState;
    desiredState.rotation = Eigen::Vector3d(0.0042443, 0.0134921, 0.948284);
    desiredState.position = Eigen::Vector3d(-0.369856, 0.29033, 0.29);
    desiredState.rotation_velocity = Eigen::Vector3d(0, 0, 0.5);
    desiredState.linear_velocity = Eigen::Vector3d(-0.220608, 0.0354218, 0);
    auto xD = convexMPC.getTrajectory(desiredState);
    auto x0 = openData<double>(getTestDataCSV("x0"));
    auto IBody = openData<double>(getTestDataCSV("IBody"));
    auto rYaw = openData<double>(getTestDataCSV("rYaw"));
    auto rFeet = openData<double>(getTestDataCSV("rFeet"));
    auto stateWeights = openData<double>(getTestDataCSV("stateWeights"));
    auto example_H = openData<double>(getTestDataCSV("H"));
    auto example_g = openData<double>(getTestDataCSV("g"));

    convexMPC.updateBodyInertia(IBody);
    convexMPC.updateBodyMass(mass);
    convexMPC.updateStateWeights(stateWeights);
    convexMPC.updateForceWeights(4e-5);
    auto Bdt = convexMPC.getBdt(rYaw, rFeet);
    auto Adt = convexMPC.getAdt(rYaw);
    auto [Aqp, Bqp] = convexMPC.getAqp_Bqp(Adt, Bdt);
    auto [H, g] = convexMPC.getH_g(x0, xD, Aqp, Bqp);

    for (int i = 0; i < numLegs * 3 * horizon; i++)
    {
        EXPECT_NEAR(g(i, 0), example_g(i, 0), ABS_ERROR);
        for (int j = 0; j < numLegs * 3 * horizon; j++)
        {
            EXPECT_NEAR(H(i, j), example_H(i, j), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, getHRed_gRed)
{
    MPCdata desiredState;
    desiredState.rotation = Eigen::Vector3d(0.0042443, 0.0134921, 0.948284);
    desiredState.position = Eigen::Vector3d(-0.369856, 0.29033, 0.29);
    desiredState.rotation_velocity = Eigen::Vector3d(0, 0, 0.5);
    desiredState.linear_velocity = Eigen::Vector3d(-0.220608, 0.0354218, 0);
    auto xD = convexMPC.getTrajectory(desiredState);
    auto x0 = openData<double>(getTestDataCSV("x0"));
    auto IBody = openData<double>(getTestDataCSV("IBody"));
    auto rYaw = openData<double>(getTestDataCSV("rYaw"));
    auto rFeet = openData<double>(getTestDataCSV("rFeet"));
    auto stateWeights = openData<double>(getTestDataCSV("stateWeights"));
    auto example_H = openData<double>(getTestDataCSV("H"));
    auto example_g = openData<double>(getTestDataCSV("g"));
    auto example_gRed = openData<double>(getTestDataCSV("gRed"));
    auto example_HRed = openData<double>(getTestDataCSV("HRed"));

    convexMPC.updateBodyInertia(IBody);
    convexMPC.updateBodyMass(mass);
    convexMPC.updateStateWeights(stateWeights);
    convexMPC.updateForceWeights(4e-5);
    auto Bdt = convexMPC.getBdt(rYaw, rFeet);
    auto Adt = convexMPC.getAdt(rYaw);
    auto [reductionVector, numLegsActive] = convexMPC.getGaitData(gaitTable);
    auto [Aqp, Bqp] = convexMPC.getAqp_Bqp(Adt, Bdt);
    auto [H, g] = convexMPC.getH_g(x0, xD, Aqp, Bqp);
    auto [HRed, gRed] = convexMPC.getHRed_gRed(H, g, reductionVector, numLegsActive);
    for (int i = 0; i < numLegsActive * 3; i++)
    {
        EXPECT_NEAR(gRed(i, 0), example_gRed(i, 0), ABS_ERROR);
        for (int j = 0; j < numLegsActive * 3; j++)
        {
            EXPECT_NEAR(HRed(i, j), example_HRed(i, j), ABS_ERROR);
        }
    }
}

TEST(ConvexMPC, solveMPC)
{
    MPCdata mpcData;
    mpcData.rotation = Eigen::Vector3d(-0.0347722, -0.0184942, 0.948284);
    mpcData.position = Eigen::Vector3d(-0.371743, 0.301919, 0.294017);
    mpcData.rotation_velocity = Eigen::Vector3d(0.0749526, 0.0135219, 0.462512);
    mpcData.linear_velocity = Eigen::Vector3d(-0.246506, 0.0248134, 0.0463022);
    MPCdata desiredState;
    desiredState.rotation = Eigen::Vector3d(0.0042443, 0.0134921, 0.948284);
    desiredState.position = Eigen::Vector3d(-0.369856, 0.29033, 0.29);
    desiredState.rotation_velocity = Eigen::Vector3d(0, 0, 0.5);
    desiredState.linear_velocity = Eigen::Vector3d(-0.220608, 0.0354218, 0);
    auto rFeet = openData<double>(getTestDataCSV("rFeet"));
    auto IBody = openData<double>(getTestDataCSV("IBody"));
    auto stateWeights = openData<double>(getTestDataCSV("stateWeights"));
    convexMPC.updateBodyInertia(IBody);
    convexMPC.updateBodyMass(mass);
    convexMPC.updateStateWeights(stateWeights);
    convexMPC.updateForceWeights(4e-5);
    convexMPC.updateMaxForce(maxForce);
    convexMPC.updateFrictionCoefficient(mu);
    auto mpcForces = convexMPC.solveMPC(mpcData, desiredState, gaitTable, rFeet);
    auto example_mpcForces = openData<double>(getTestDataCSV("mpcForces"));
    // for (int i = 0; i < numLegs * 3; i++)
    // {
    //     EXPECT_NEAR(mpcForces[i], example_mpcForces(i, 0), ABS_ERROR);
    // }

}