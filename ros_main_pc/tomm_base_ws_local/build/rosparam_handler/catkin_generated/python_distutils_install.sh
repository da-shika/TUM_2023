#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/rosparam_handler"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/genki/ros/workspaces/tomm_base_ws_local/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/genki/ros/workspaces/tomm_base_ws_local/install/lib/python3/dist-packages:/home/genki/ros/workspaces/tomm_base_ws_local/build/rosparam_handler/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/genki/ros/workspaces/tomm_base_ws_local/build/rosparam_handler" \
    "/usr/bin/python3" \
    "/home/genki/ros/workspaces/tomm_base_ws_local/src/utilities/rosparam_handler/setup.py" \
    egg_info --egg-base /home/genki/ros/workspaces/tomm_base_ws_local/build/rosparam_handler \
    build --build-base "/home/genki/ros/workspaces/tomm_base_ws_local/build/rosparam_handler" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/genki/ros/workspaces/tomm_base_ws_local/install" --install-scripts="/home/genki/ros/workspaces/tomm_base_ws_local/install/bin"
