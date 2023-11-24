# rosparam_handler

Set of scripts to auto generate parameter classes that can load parameters eigher form the ros parameter server or through reconfigure.

### Usage

1. Create a /cfg folder and add the new config file `Behavior.params` with content

    ```
    #!/usr/bin/env python
    from rosparam_handler.parameter_generator_catkin import *
    gen = ParameterGenerator()

    gen.add_parameter_type(package="control_core", header="types.h", namespace="cc", primitve_type="double", is_eigen=True)

    gen.add("tasks", paramtype="std::vector<std::string>", configurable=True)

    #Syntax : Package, Node, Config Name
    exit(gen.generate("TODO_my_package_name", "TODO_my_node_name", "Behavior"))
    ```

2. Make the file executable `chmod a+x` and add a new file `setup.py` with

    ```
    ## DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    # note: we don't have any python files in this package
    setup_args = generate_distutils_setup(
        packages=[''],
        package_data={'': ['']},
        package_dir={'': ''})

    setup(**setup_args)
    ```

3. Modify the `CMakeList.txt`

    ```
    find_package(catkin REQUIRED COMPONENTS
      ...
      rosparam_handler
      dynamic_reconfigure
    )
    ...
    catkin_python_setup()
    ...
    ## Generate dynamic reconfigure parameters in the 'cfg' folder
    generate_ros_parameter_files(
      cfg/Behavior.params
    }
    ...
    add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} TODO_my_package_name_gencfg)
    ```

4. Use the header in your code 

    ```
    #include <TODO_my_package_name/BehaviorParameters.h>
    ```