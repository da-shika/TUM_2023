# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "control_core;control_core_msgs;dynamic_reconfigure;roscpp;rosparam_handler;urdf;ics_tsid_common;skin_model".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lics_robot_wrapper".split(';') if "-lics_robot_wrapper" != "" else []
PROJECT_NAME = "ics_robot_wrapper"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"