cmake_minimum_required(VERSION 3.10)
project(EKF-INS)

set(CMAKE_CXX_STANDARD 17)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    set(EIGEN_PACKAGE Eigen)
endif()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

file(GLOB EKF_INCLUDE
        "include/*.h")

file(GLOB EKF_SRC
        "src/*.cpp")

add_library(EKF-INS SHARED ${EKF_SRC})
target_include_directories(EKF-INS PRIVATE ${EKF_INCLUDE})