execute_process(COMMAND "/home/genki/ros/workspace/pr2_ws/build/experiment_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/genki/ros/workspace/pr2_ws/build/experiment_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
