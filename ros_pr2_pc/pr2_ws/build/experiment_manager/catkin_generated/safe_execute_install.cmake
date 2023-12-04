execute_process(COMMAND "/home/chen/ros/workspace_genki/pr2_ws/build/experiment_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/chen/ros/workspace_genki/pr2_ws/build/experiment_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
