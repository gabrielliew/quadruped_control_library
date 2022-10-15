cmake_minimum_required(VERSION 3.16)

PROJECT(quadruped_control CXX)
SET(PACKAGE_NAME "quadruped_control_library")
SET(PACKAGE_VERSION "0.1")
SET(PACKAGE_SO_VERSION "0.1")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_${lang}_FLAGS_RELEASE_INIT "-O3 -DNDEBUG")
set(CMAKE_CXX_STANDARD 17)

# find_package(catkin REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
FIND_PACKAGE(RBDL COMPONENTS URDFREADER REQUIRED)

# catkin_package(
# INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS
# DEPENDS Eigen3::Eigen qpOASES rbdl
# )
IF(NOT CMAKE_VERBOSE_MAKEFILE)
  SET(CMAKE_VERBOSE_MAKEFILE OFF)
ENDIF(NOT CMAKE_VERBOSE_MAKEFILE)

option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)

include_directories(
  include

  # ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE QUADRUPED_CONTROL_SOURCES "src/*.cpp")

SET(QUADRUPED_CONTROL_BUILD_STATIC_DEFAULT OFF)
OPTION(QUADRUPED_CONTROL_BUILD_STATIC "Build statically linked library (otherwise dynamiclly linked)" ${QUADRUPED_CONTROL_BUILD_STATIC_DEFAULT})

IF(QUADRUPED_CONTROL_BUILD_STATIC)
  ADD_LIBRARY(quadruped_control STATIC ${QUADRUPED_CONTROL_SOURCES})
ELSE(QUADRUPED_CONTROL_BUILD_STATIC)
  ADD_LIBRARY(quadruped_control SHARED ${QUADRUPED_CONTROL_SOURCES})
ENDIF(QUADRUPED_CONTROL_BUILD_STATIC)

TARGET_INCLUDE_DIRECTORIES(quadruped_control PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_SOURCE_DIR}/test
)

target_link_libraries(quadruped_control
  Eigen3::Eigen
  ${qpOASES_LIBRARY}
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)

INSTALL(TARGETS quadruped_control
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
file(GLOB_RECURSE test_sources "test/test_*.cpp")
add_executable(test_main ${test_sources})
target_link_libraries(test_main
  Eigen3::Eigen
  ${qpOASES_LIBRARY}
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  quadruped_control
  GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(test_main)
