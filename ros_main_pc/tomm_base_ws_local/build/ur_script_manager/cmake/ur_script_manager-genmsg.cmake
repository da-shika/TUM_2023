# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ur_script_manager: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ur_script_manager_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_custom_target(_ur_script_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur_script_manager" "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" ""
)

get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_custom_target(_ur_script_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur_script_manager" "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur_script_manager
)
_generate_srv_cpp(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur_script_manager
)

### Generating Module File
_generate_module_cpp(ur_script_manager
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur_script_manager
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ur_script_manager_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ur_script_manager_generate_messages ur_script_manager_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_cpp _ur_script_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_cpp _ur_script_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur_script_manager_gencpp)
add_dependencies(ur_script_manager_gencpp ur_script_manager_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur_script_manager_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur_script_manager
)
_generate_srv_eus(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur_script_manager
)

### Generating Module File
_generate_module_eus(ur_script_manager
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur_script_manager
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ur_script_manager_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ur_script_manager_generate_messages ur_script_manager_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_eus _ur_script_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_eus _ur_script_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur_script_manager_geneus)
add_dependencies(ur_script_manager_geneus ur_script_manager_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur_script_manager_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur_script_manager
)
_generate_srv_lisp(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur_script_manager
)

### Generating Module File
_generate_module_lisp(ur_script_manager
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur_script_manager
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ur_script_manager_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ur_script_manager_generate_messages ur_script_manager_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_lisp _ur_script_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_lisp _ur_script_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur_script_manager_genlisp)
add_dependencies(ur_script_manager_genlisp ur_script_manager_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur_script_manager_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur_script_manager
)
_generate_srv_nodejs(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur_script_manager
)

### Generating Module File
_generate_module_nodejs(ur_script_manager
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur_script_manager
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ur_script_manager_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ur_script_manager_generate_messages ur_script_manager_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_nodejs _ur_script_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_nodejs _ur_script_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur_script_manager_gennodejs)
add_dependencies(ur_script_manager_gennodejs ur_script_manager_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur_script_manager_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager
)
_generate_srv_py(ur_script_manager
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager
)

### Generating Module File
_generate_module_py(ur_script_manager
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ur_script_manager_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ur_script_manager_generate_messages ur_script_manager_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/getScriptManagerStates.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_py _ur_script_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/tomm-robot/ur_script_manager/srv/setScriptManagerState.srv" NAME_WE)
add_dependencies(ur_script_manager_generate_messages_py _ur_script_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur_script_manager_genpy)
add_dependencies(ur_script_manager_genpy ur_script_manager_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur_script_manager_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur_script_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur_script_manager
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ur_script_manager_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur_script_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur_script_manager
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ur_script_manager_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur_script_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur_script_manager
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ur_script_manager_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur_script_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur_script_manager
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ur_script_manager_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur_script_manager
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ur_script_manager_generate_messages_py std_msgs_generate_messages_py)
endif()
