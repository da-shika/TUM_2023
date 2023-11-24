cmake_minimum_required(VERSION 3.0.2)
project(tomm_hardware_real)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  soem
  tf
  roslib  #for ros::package::getPath()
  ur_script_manager
  message_generation
  yaml_parameters
)

##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

##############################################################################
# TUM-ICS Math Lib
##############################################################################
find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})

##############################################################################
# RT Threads
##############################################################################
find_package(RtThreads REQUIRED)
add_definitions(${RTTHREADS_DEFINITIONS})

##############################################################################
# QT5
##############################################################################
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)


add_message_files(
  FILES
  DriverState.msg
  OmniBaseState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tomm_hardware_real
 CATKIN_DEPENDS controller_manager hardware_interface roscpp soem tf roslib ur_script_manager message_runtime yaml_parameters
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TMATH_INCLUDE_DIRS}
  ${RTTHREADS_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/tomm_hardware_real.cpp
  src/arm/arm_configs.cpp
  # src/arm/arm_yaml_configs.cpp
  src/arm/arm_comm_interface.cpp
  src/arm/arm_interface.cpp
  src/omnibase/omnibase_configs.cpp
  src/omnibase/omnibase_rt_driver.cpp
  src/omnibase/omnibase_interface.cpp
  src/utilities/data_unpacker.cpp
  src/utilities/robot_state_client.cpp
  src/utilities/tcp_socket_tools.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${RTTHREADS_LIBRARIES}
  ${TMATH_LIBRARIES}
  Qt5::Core 
  Qt5::Network 
  Qt5::Widgets
)

set(TEST_TARGETS
  test_joint_state
  test_arm_configs_loading
  test_base_configs_loading
  test_arm_interface
  test_base_interface
)

foreach(test_target ${TEST_TARGETS})
  add_executable(${test_target} 
    src/tests/${test_target}.cpp)

  target_link_libraries(${test_target}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    Qt5::Core 
    Qt5::Network 
    Qt5::Widgets)
endforeach()

set(APP_TARGETS
  tomm_hardware_real_node
)

foreach(app_target ${APP_TARGETS})
  add_executable(${app_target} 
    src/applications/${app_target}.cpp)

  target_link_libraries(${app_target}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
    Qt5::Core 
    Qt5::Network 
    Qt5::Widgets)
endforeach()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tomm_hardware_real.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
