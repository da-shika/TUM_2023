# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;ics_tsid_wrapper;tomm_core;tomm_interfaces;walking_core".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltomm_commanded_robot".split(';') if "-ltomm_commanded_robot" != "" else []
PROJECT_NAME = "tomm_commanded_robot"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"
