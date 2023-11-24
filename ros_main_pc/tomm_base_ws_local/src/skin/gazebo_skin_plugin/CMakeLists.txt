cmake_minimum_required(VERSION 2.8.3)
project(gazebo_skin_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  pcl_conversions
  pcl_ros
  tf
  tf_conversions
  skin_client
  control_core_msgs
)

find_package (gazebo REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp gazebo_custom_sensor_preloader pcl_conversions pcl_ros tf tf_conversions control_core_msgs skin_client
#  DEPENDS pugixml
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
                     ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/skin_sensor_plugin.cpp
  src/skin_sensor.cpp)

add_library(register_${PROJECT_NAME} 
  src/register_skin_sensor.cpp
  src/skin_sensor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${PCL_LIBRARIES})

target_link_libraries(register_${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${PCL_LIBRARIES})


install(FILES skin_sensor.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})