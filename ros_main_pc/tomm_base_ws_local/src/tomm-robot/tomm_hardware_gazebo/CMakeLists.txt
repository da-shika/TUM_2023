cmake_minimum_required(VERSION 2.8.3)
project(tomm_hardware_gazebo)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  hardware_interface
  joint_limits_interface
  gazebo_ros_control
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
    INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
    CATKIN_DEPENDS control_toolbox hardware_interface joint_limits_interface gazebo_ros_control
    DEPENDS gazebo Eigen3
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
  src/tomm_hardware_gazebo.cpp
  src/tomm_default_robot_hw_sim.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
install (FILES tomm_hardware_gazebo_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
