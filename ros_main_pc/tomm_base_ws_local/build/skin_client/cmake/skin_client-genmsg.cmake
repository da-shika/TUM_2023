# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "skin_client: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iskin_client:/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(skin_client_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_custom_target(_skin_client_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "skin_client" "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(skin_client
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/skin_client
)

### Generating Services

### Generating Module File
_generate_module_cpp(skin_client
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/skin_client
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(skin_client_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(skin_client_generate_messages skin_client_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_dependencies(skin_client_generate_messages_cpp _skin_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(skin_client_gencpp)
add_dependencies(skin_client_gencpp skin_client_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS skin_client_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(skin_client
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/skin_client
)

### Generating Services

### Generating Module File
_generate_module_eus(skin_client
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/skin_client
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(skin_client_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(skin_client_generate_messages skin_client_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_dependencies(skin_client_generate_messages_eus _skin_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(skin_client_geneus)
add_dependencies(skin_client_geneus skin_client_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS skin_client_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(skin_client
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/skin_client
)

### Generating Services

### Generating Module File
_generate_module_lisp(skin_client
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/skin_client
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(skin_client_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(skin_client_generate_messages skin_client_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_dependencies(skin_client_generate_messages_lisp _skin_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(skin_client_genlisp)
add_dependencies(skin_client_genlisp skin_client_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS skin_client_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(skin_client
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/skin_client
)

### Generating Services

### Generating Module File
_generate_module_nodejs(skin_client
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/skin_client
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(skin_client_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(skin_client_generate_messages skin_client_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_dependencies(skin_client_generate_messages_nodejs _skin_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(skin_client_gennodejs)
add_dependencies(skin_client_gennodejs skin_client_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS skin_client_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(skin_client
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/skin_client
)

### Generating Services

### Generating Module File
_generate_module_py(skin_client
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/skin_client
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(skin_client_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(skin_client_generate_messages skin_client_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/skin/skin_client/msg/SkinPatchData.msg" NAME_WE)
add_dependencies(skin_client_generate_messages_py _skin_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(skin_client_genpy)
add_dependencies(skin_client_genpy skin_client_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS skin_client_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/skin_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/skin_client
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(skin_client_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/skin_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/skin_client
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(skin_client_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/skin_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/skin_client
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(skin_client_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/skin_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/skin_client
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(skin_client_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/skin_client)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/skin_client\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/skin_client
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(skin_client_generate_messages_py std_msgs_generate_messages_py)
endif()
