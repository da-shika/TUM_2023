# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "tomm_commanded_robot;ics_formulation;roscpp;rosparam_handler".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltomm_whole_body_controller".split(';') if "-ltomm_whole_body_controller" != "" else []
PROJECT_NAME = "tomm_whole_body_controller"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"