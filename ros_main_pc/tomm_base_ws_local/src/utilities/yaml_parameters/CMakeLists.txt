cmake_minimum_required(VERSION 3.0.2)
project(yaml_parameters)

# add_compile_options(-std=c++11)

## Package dependencies
find_package(catkin REQUIRED COMPONENTS
  roslib
)
find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)   # find_package(yaml-cpp) only set YAML_CPP_LIBRARIES to "yaml-cpp"
set(YAML_CPP_INCLUDE_DIR "/usr/local/include")
set(YAML_CPP_LIBRARIES "/usr/local/lib/libyaml-cpp.so")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Package configuration (this information is passed to projects/packages that
# depend on this package)
catkin_package(
    INCLUDE_DIRS include ${YAML_CPP_INCLUDE_DIR}
    LIBRARIES ${YAML_CPP_LIBRARIES}
    CATKIN_DEPENDS roslib
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/yaml_parameters.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${yaml_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(test_parameter_loading 
#   src/tests/test_parameter_loading.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(test_parameter_loading
#   ${YAML_CPP_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

set(TEST_TARGETS
  test_parameter_loading
  test_namespaces
)
foreach(target ${TEST_TARGETS})
  add_executable(${target} src/tests/${target}.cpp)
  target_link_libraries(${target}
      ${YAML_CPP_LIBRARIES}
      ${catkin_LIBRARIES}
  )
endforeach()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html#

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)