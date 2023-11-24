# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tomm_hardware_real: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itomm_hardware_real:/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tomm_hardware_real_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_custom_target(_tomm_hardware_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tomm_hardware_real" "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" ""
)

get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_custom_target(_tomm_hardware_real_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tomm_hardware_real" "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" "tomm_hardware_real/DriverState"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomm_hardware_real
)
_generate_msg_cpp(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg"
  "${MSG_I_FLAGS}"
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomm_hardware_real
)

### Generating Services

### Generating Module File
_generate_module_cpp(tomm_hardware_real
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomm_hardware_real
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tomm_hardware_real_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tomm_hardware_real_generate_messages tomm_hardware_real_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_cpp _tomm_hardware_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_cpp _tomm_hardware_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomm_hardware_real_gencpp)
add_dependencies(tomm_hardware_real_gencpp tomm_hardware_real_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomm_hardware_real_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomm_hardware_real
)
_generate_msg_eus(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg"
  "${MSG_I_FLAGS}"
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomm_hardware_real
)

### Generating Services

### Generating Module File
_generate_module_eus(tomm_hardware_real
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomm_hardware_real
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tomm_hardware_real_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tomm_hardware_real_generate_messages tomm_hardware_real_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_eus _tomm_hardware_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_eus _tomm_hardware_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomm_hardware_real_geneus)
add_dependencies(tomm_hardware_real_geneus tomm_hardware_real_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomm_hardware_real_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomm_hardware_real
)
_generate_msg_lisp(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg"
  "${MSG_I_FLAGS}"
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomm_hardware_real
)

### Generating Services

### Generating Module File
_generate_module_lisp(tomm_hardware_real
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomm_hardware_real
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tomm_hardware_real_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tomm_hardware_real_generate_messages tomm_hardware_real_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_lisp _tomm_hardware_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_lisp _tomm_hardware_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomm_hardware_real_genlisp)
add_dependencies(tomm_hardware_real_genlisp tomm_hardware_real_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomm_hardware_real_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomm_hardware_real
)
_generate_msg_nodejs(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg"
  "${MSG_I_FLAGS}"
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomm_hardware_real
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tomm_hardware_real
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomm_hardware_real
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tomm_hardware_real_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tomm_hardware_real_generate_messages tomm_hardware_real_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_nodejs _tomm_hardware_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_nodejs _tomm_hardware_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomm_hardware_real_gennodejs)
add_dependencies(tomm_hardware_real_gennodejs tomm_hardware_real_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomm_hardware_real_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real
)
_generate_msg_py(tomm_hardware_real
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg"
  "${MSG_I_FLAGS}"
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real
)

### Generating Services

### Generating Module File
_generate_module_py(tomm_hardware_real
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tomm_hardware_real_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tomm_hardware_real_generate_messages tomm_hardware_real_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/DriverState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_py _tomm_hardware_real_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/tomm_hardware_real/msg/OmniBaseState.msg" NAME_WE)
add_dependencies(tomm_hardware_real_generate_messages_py _tomm_hardware_real_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomm_hardware_real_genpy)
add_dependencies(tomm_hardware_real_genpy tomm_hardware_real_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomm_hardware_real_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomm_hardware_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomm_hardware_real
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tomm_hardware_real_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomm_hardware_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomm_hardware_real
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tomm_hardware_real_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomm_hardware_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomm_hardware_real
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tomm_hardware_real_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomm_hardware_real)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomm_hardware_real
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tomm_hardware_real_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomm_hardware_real
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tomm_hardware_real_generate_messages_py std_msgs_generate_messages_py)
endif()
