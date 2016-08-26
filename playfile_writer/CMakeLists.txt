cmake_minimum_required(VERSION 2.8.3)
project(playfile_writer)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# Executables 
cs_add_executable(playfile_writer src/playfile_writer.cpp)

#target_link_libraries(playfile_jointspace davinci_joint_publisher playfile_record davinci_kinematics ${catkin_LIBRARIES})
cs_install()
cs_export()
