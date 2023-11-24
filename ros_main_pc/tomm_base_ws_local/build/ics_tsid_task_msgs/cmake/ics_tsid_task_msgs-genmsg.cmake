# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ics_tsid_task_msgs: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iics_tsid_task_msgs:/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg;-Icontrol_core_msgs:/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iwalking_core_msgs:/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/walking_core_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ics_tsid_task_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_custom_target(_ics_tsid_task_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ics_tsid_task_msgs" "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" "std_msgs/Int32:std_msgs/Float64:control_core_msgs/Vector:std_msgs/Int64"
)

get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_custom_target(_ics_tsid_task_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ics_tsid_task_msgs" "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" "std_msgs/Int32:std_msgs/Float64:control_core_msgs/Vector:std_msgs/Int64"
)

get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_custom_target(_ics_tsid_task_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ics_tsid_task_msgs" "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" "std_msgs/Int32:std_msgs/Float64:control_core_msgs/Vector:std_msgs/Int64"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_cpp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_cpp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(ics_tsid_task_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ics_tsid_task_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ics_tsid_task_msgs_generate_messages ics_tsid_task_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_cpp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_cpp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_cpp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ics_tsid_task_msgs_gencpp)
add_dependencies(ics_tsid_task_msgs_gencpp ics_tsid_task_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ics_tsid_task_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_eus(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_eus(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(ics_tsid_task_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ics_tsid_task_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ics_tsid_task_msgs_generate_messages ics_tsid_task_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_eus _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_eus _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_eus _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ics_tsid_task_msgs_geneus)
add_dependencies(ics_tsid_task_msgs_geneus ics_tsid_task_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ics_tsid_task_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_lisp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_lisp(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(ics_tsid_task_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ics_tsid_task_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ics_tsid_task_msgs_generate_messages ics_tsid_task_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_lisp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_lisp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_lisp _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ics_tsid_task_msgs_genlisp)
add_dependencies(ics_tsid_task_msgs_genlisp ics_tsid_task_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ics_tsid_task_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_nodejs(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_nodejs(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ics_tsid_task_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ics_tsid_task_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ics_tsid_task_msgs_generate_messages ics_tsid_task_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ics_tsid_task_msgs_gennodejs)
add_dependencies(ics_tsid_task_msgs_gennodejs ics_tsid_task_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ics_tsid_task_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_py(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs
)
_generate_msg_py(ics_tsid_task_msgs
  "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/control_core_msgs/msg/Vector.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(ics_tsid_task_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ics_tsid_task_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ics_tsid_task_msgs_generate_messages ics_tsid_task_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinDistanceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_py _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SkinForceConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_py _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_task_msgs/msg/SelfCollisionConstraint.msg" NAME_WE)
add_dependencies(ics_tsid_task_msgs_generate_messages_py _ics_tsid_task_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ics_tsid_task_msgs_genpy)
add_dependencies(ics_tsid_task_msgs_genpy ics_tsid_task_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ics_tsid_task_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ics_tsid_task_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET control_core_msgs_generate_messages_cpp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_cpp control_core_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET walking_core_msgs_generate_messages_cpp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_cpp walking_core_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ics_tsid_task_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET control_core_msgs_generate_messages_eus)
  add_dependencies(ics_tsid_task_msgs_generate_messages_eus control_core_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ics_tsid_task_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET walking_core_msgs_generate_messages_eus)
  add_dependencies(ics_tsid_task_msgs_generate_messages_eus walking_core_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ics_tsid_task_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET control_core_msgs_generate_messages_lisp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_lisp control_core_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET walking_core_msgs_generate_messages_lisp)
  add_dependencies(ics_tsid_task_msgs_generate_messages_lisp walking_core_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ics_tsid_task_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET control_core_msgs_generate_messages_nodejs)
  add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs control_core_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET walking_core_msgs_generate_messages_nodejs)
  add_dependencies(ics_tsid_task_msgs_generate_messages_nodejs walking_core_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ics_tsid_task_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET control_core_msgs_generate_messages_py)
  add_dependencies(ics_tsid_task_msgs_generate_messages_py control_core_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ics_tsid_task_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET walking_core_msgs_generate_messages_py)
  add_dependencies(ics_tsid_task_msgs_generate_messages_py walking_core_msgs_generate_messages_py)
endif()
