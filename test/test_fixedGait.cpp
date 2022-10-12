#include <gtest/gtest.h>

#include "quadruped_control_library/fixedGait.hpp"

#define ABS_ERROR 0.00001

FixedGait fixedGait(4, 10, 0.6, 0.3, std::vector<int>{0, 5, 5, 0});

TEST(FixedGait, getSwingProgression)
{
    auto swingResult = fixedGait.getSwingProgression(14.3 * 1e9); // 0.5s passed
    EXPECT_NEAR(swingResult[0], 0.66666666, ABS_ERROR);
    EXPECT_NEAR(swingResult[1], 0, ABS_ERROR);
    EXPECT_NEAR(swingResult[2], 0, ABS_ERROR);
    EXPECT_NEAR(swingResult[3], 0.66666666, ABS_ERROR);
}

TEST(FixedGait, getContactProgression)
{
    auto contactResult = fixedGait.getContactProgression(14.3 * 1e9);
    EXPECT_NEAR(contactResult[0], 0, ABS_ERROR);
    EXPECT_NEAR(contactResult[1], 0.66666666, ABS_ERROR);
    EXPECT_NEAR(contactResult[2], 0.66666666, ABS_ERROR);
    EXPECT_NEAR(contactResult[3], 0, ABS_ERROR);
}

TEST(FixedGait, getGaitTable)
{
    auto tableResult = fixedGait.getGaitTable(14.3 * 1e9);
    Eigen::Matrix<int, 10, 4> tableData;
    tableData << 0, 1, 1, 0,
        0, 1, 1, 0,
        1, 0, 0, 1,
        1, 0, 0, 1,
        1, 0, 0, 1,
        1, 0, 0, 1,
        1, 0, 0, 1,
        0, 1, 1, 0,
        0, 1, 1, 0,
        0, 1, 1, 0;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            EXPECT_EQ(tableResult(i, j), tableData(i, j));
        }
    }
}

#ifdef NDEBUG
TEST(FixedGait, wrongNumberLegs)
{
    ASSERT_DEATH(FixedGait(5, 10, 0.6, 0.3, std::vector<int>{0, 5, 5, 0}), "");
}
#endif
