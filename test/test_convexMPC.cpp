#include <gtest/gtest.h>

#include "helper.hpp"
#include "save_load_eigen_csv.hpp"
#include "quadruped_control_library/convexMPC.hpp"
#include <iostream>

#define ABS_ERROR 0.001

int horizon = 10;
int numLegs = 4;
int activeLegs = 4;
double maxForce = 120;
double mu = 0.4;
double mass = 9;
std::vector<bool> gaitTable = {0, 1, 1, 0,
                               0, 1, 1, 0,
                               0, 1, 1, 0,
                               0, 1, 1, 0,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               1, 0, 0, 1,
                               0, 1, 1, 0};

MPCdata desiredState;

ConvexMPC convexMPC(10, 0.026, -9.8, numLegs);

TEST(ConvexMPC, getLowerBoundary)
{
    auto lB = convexMPC.getLowerBoundary(activeLegs); // 0.5s passed
    EXPECT_EQ(lB.size(), activeLegs * 5);
}

TEST(ConvexMPC, getUpperBoundary)
{
    auto uB = convexMPC.getUpperBoundary(activeLegs, maxForce); // 0.5s passed
    for (int i = 0; i < activeLegs; i++)
    {
        EXPECT_EQ(uB[i * 5 + 0], std::numeric_limits<double>::max());
        EXPECT_EQ(uB[i * 5 + 1], std::numeric_limits<double>::max());
        EXPECT_EQ(uB[i * 5 + 2], std::numeric_limits<double>::max());
        EXPECT_EQ(uB[i * 5 + 3], std::numeric_limits<double>::max());
        EXPECT_EQ(uB[i * 5 + 4], maxForce);
    }
    EXPECT_EQ(uB.size(), activeLegs * 5);
}

TEST(ConvexMPC, getConstraintMatrix)
{
    auto constraintMatrix = convexMPC.getConstraintMatrix(activeLegs, mu); // 0.5s passed
    for (int i = 0; i < activeLegs; i++)
    {
        EXPECT_EQ(constraintMatrix(i * 5 + 0, i * 3 + 0), 1.0 / mu);
        EXPECT_EQ(constraintMatrix(i * 5 + 1, i * 3 + 0), -1.0 / mu);
        EXPECT_EQ(constraintMatrix(i * 5 + 2, i * 3 + 1), 1.0 / mu);
        EXPECT_EQ(constraintMatrix(i * 5 + 3, i * 3 + 1), -1.0 / mu);
        EXPECT_EQ(constraintMatrix(i * 5 + 0, i * 3 + 2), 1.0);
        EXPECT_EQ(constraintMatrix(i * 5 + 1, i * 3 + 2), 1.0);
        EXPECT_EQ(constraintMatrix(i * 5 + 2, i * 3 + 2), 1.0);
        EXPECT_EQ(constraintMatrix(i * 5 + 3, i * 3 + 2), 1.0);
        EXPECT_EQ(constraintMatrix(i * 5 + 4, i * 3 + 2), 1.0);
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

// TEST(ConvexMPC, getTrajectory)
// {
//     desiredState auto xD = convexMPC.getTrajectory(desiredState);
// }

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
    }
    for (int i = 0; i < 13 * horizon; i++)
    {
        for (int j = 0; j < numLegs * 3 * horizon; j++)
        {
            EXPECT_NEAR(Bqp(i, j), example_Bqp(i, j), ABS_ERROR);
        }
    }
}
