#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

#include "path.hpp"
#include <string>

std::string getTestDataCSV(std::string testData) {
  std::string temp = source_directory + std::string("test/data/") + testData +
                     std::string(".csv");
  return temp;
}

#endif /* TEST_HELPER_HPP */
