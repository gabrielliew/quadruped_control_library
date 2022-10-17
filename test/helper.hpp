#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

#include <string>
#include "path.hpp"

std::string getTestDataCSV(std::string testData)
{
    std::string temp = source_directory + std::string("test/data/") + testData + std::string(".csv");
    return temp;
}

#endif /* TEST_HELPER_HPP */
