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

echo_and_run cd "/home/genki/ros/workspace/pr2_ws/src/test"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/genki/ros/workspace/pr2_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/genki/ros/workspace/pr2_ws/install/lib/python3/dist-packages:/home/genki/ros/workspace/pr2_ws/build/test/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/genki/ros/workspace/pr2_ws/build/test" \
    "/usr/bin/python3" \
    "/home/genki/ros/workspace/pr2_ws/src/test/setup.py" \
     \
    build --build-base "/home/genki/ros/workspace/pr2_ws/build/test" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/genki/ros/workspace/pr2_ws/install" --install-scripts="/home/genki/ros/workspace/pr2_ws/install/bin"