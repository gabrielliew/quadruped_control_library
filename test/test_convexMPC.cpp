#include <gtest/gtest.h>

#include "quadruped_control_library/convexMPC.hpp"
#include <iostream>

ConvexMPC convexMPC(10, 0.3, -9.8, 4);

TEST(ConvexMPC, getUpperLimit)
{
    EXPECT_EQ(1, 1);
    auto upperLimit = convexMPC.getLowerBoundary(2); // 0.5s passed
    std::cout << upperLimit << std::endl;
}

TEST(ConvexMPC, getConstraintMatrix)
{
    EXPECT_EQ(1, 1);
    auto constraintMatrix = convexMPC.getConstraintMatrix(2, 0.5); // 0.5s passed
    std::cout << constraintMatrix << std::endl;
}

// TEST(ConvexMPC, getHRed_gRed)
// {

// }