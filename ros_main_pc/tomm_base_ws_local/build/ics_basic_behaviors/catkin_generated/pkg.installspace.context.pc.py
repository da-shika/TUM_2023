# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;ics_behavior;roscpp;rosparam_handler".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lics_basic_behaviors".split(';') if "-lics_basic_behaviors" != "" else []
PROJECT_NAME = "ics_basic_behaviors"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"