# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_manager;hardware_interface;roscpp;soem;tf;roslib;ur_script_manager;message_runtime;yaml_parameters".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltomm_hardware_real".split(';') if "-ltomm_hardware_real" != "" else []
PROJECT_NAME = "tomm_hardware_real"
PROJECT_SPACE_DIR = "/home/genki/ros/workspaces/tomm_base_ws_local/install"
PROJECT_VERSION = "0.0.0"
