cmake_minimum_required(VERSION 2.8.3)
project(davinci_interface)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# Libraries
cs_add_library(davinci_interface src/davinci_interface.cpp)   

cs_install()
cs_export()
