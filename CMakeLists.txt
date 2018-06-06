cmake_minimum_required(VERSION 3.10)
project(EKF-INS)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)

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

find_package(PythonLibs 2.7)

include_directories(include ${EIGEN3_INCLUDE_DIRS} 3rdparty)

file(GLOB EKF_INCLUDE
        "include/*.h")

file(GLOB EKF_SRC
        "src/*.cpp")

add_library(EKF-INS SHARED ${EKF_SRC})
target_include_directories(EKF-INS PRIVATE ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(EKF-INS ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})