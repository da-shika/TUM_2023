execute_process(COMMAND "/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/genki/ros/workspaces/tomm_base_ws_local/build/ics_tsid_tasks/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
