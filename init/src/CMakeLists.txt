cmake_minimum_required (VERSION 3.0.0)
project (Transformation)

# find_package(Eigen3 REQUIRED)
include_directories("/usr/local/Cellar/eigen/3.4.0_1/include/eigen3/")

add_executable (Transformation main.cpp)
