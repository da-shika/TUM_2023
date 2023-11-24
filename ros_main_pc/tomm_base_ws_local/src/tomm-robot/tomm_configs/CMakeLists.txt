cmake_minimum_required(VERSION 2.8.3)
project(tomm_configs)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

foreach(dir Arms Cameras Hands Knowledge LacqueyGrippers ScriptManager Skills)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)




