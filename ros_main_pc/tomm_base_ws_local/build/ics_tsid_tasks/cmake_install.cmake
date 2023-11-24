# Install script for directory: /home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_tasks

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/genki/ros/workspaces/tomm_base_ws_local/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE PROGRAM FILES "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE PROGRAM FILES "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/setup.bash;/home/genki/ros/workspaces/tomm_base_ws_local/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE FILE FILES
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/setup.bash"
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/setup.sh;/home/genki/ros/workspaces/tomm_base_ws_local/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE FILE FILES
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/setup.sh"
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/setup.zsh;/home/genki/ros/workspaces/tomm_base_ws_local/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE FILE FILES
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/setup.zsh"
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki/ros/workspaces/tomm_base_ws_local/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/genki/ros/workspaces/tomm_base_ws_local/install" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinComplianceParameters.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinDistanceConstraintsParameters.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinForceConstraintsParameters.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/param")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/ics_tsid_tasks" TYPE DIRECTORY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/param")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinComplianceConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinDistanceConstraintsConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/include/ics_tsid_tasks/TaskSkinForceConstraintsConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/ics_tsid_tasks" TYPE DIRECTORY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/python3/dist-packages/ics_tsid_tasks/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/ics_tsid_tasks.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ics_tsid_tasks/cmake" TYPE FILE FILES
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/ics_tsid_tasksConfig.cmake"
    "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/installspace/ics_tsid_tasksConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ics_tsid_tasks" TYPE FILE FILES "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_tasks/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/libics_tsid_tasks.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so"
         OLD_RPATH "/opt/openrobots/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_common/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/control_core/lib:/opt/ros/noetic/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/skin_client/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE DIRECTORY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_tasks/include/ics_tsid_tasks/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_tasks/lib/libics_tsid_tasks.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so"
         OLD_RPATH "/opt/openrobots/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/ics_tsid_common/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/control_core/lib:/opt/ros/noetic/lib:/home/genki/ros/workspaces/tomm_base_ws_local/devel/.private/skin_client/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libics_tsid_tasks.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ics_tsid_tasks" TYPE DIRECTORY FILES "/home/genki/ros/workspaces/tomm_base_ws_local/src/ics/ics_tsid_tasks/include/ics_tsid_tasks/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/\\.svn$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
