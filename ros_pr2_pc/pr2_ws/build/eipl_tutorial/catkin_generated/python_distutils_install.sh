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

echo_and_run cd "/home/chen/ros/workspace_genki/pr2_ws/src/eipl_tutorial"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/chen/ros/workspace_genki/pr2_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/chen/ros/workspace_genki/pr2_ws/install/lib/python2.7/dist-packages:/home/chen/ros/workspace_genki/pr2_ws/build/eipl_tutorial/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/chen/ros/workspace_genki/pr2_ws/build/eipl_tutorial" \
    "/usr/bin/python2" \
    "/home/chen/ros/workspace_genki/pr2_ws/src/eipl_tutorial/setup.py" \
     \
    build --build-base "/home/chen/ros/workspace_genki/pr2_ws/build/eipl_tutorial" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/chen/ros/workspace_genki/pr2_ws/install" --install-scripts="/home/chen/ros/workspace_genki/pr2_ws/install/bin"
