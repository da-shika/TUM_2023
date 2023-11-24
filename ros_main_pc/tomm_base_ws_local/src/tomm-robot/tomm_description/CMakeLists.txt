cmake_minimum_required(VERSION 2.8.3)
project(tomm_description)

find_package(catkin REQUIRED)

catkin_package()

# include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
# ics_ros_debgen()

foreach(dir config meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
