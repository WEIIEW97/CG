cmake_minimum_required(VERSION 3.10)
project(src)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/Cellar/eigen/3.4.0_1/include/eigen3/")

add_executable(src main.cpp)

target_link_libraries(src ${OpenCV_LIBS})
