# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;ics_basic_behaviors;tomm_basic_behaviors;tomm_whole_body_controller;tomm_hardware_interface".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltomm_application".split(';') if "-ltomm_application" != "" else []
PROJECT_NAME = "tomm_application"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"
