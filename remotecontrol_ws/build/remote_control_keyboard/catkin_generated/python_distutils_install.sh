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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control_keyboard"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/install/lib/python2.7/dist-packages:/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/build" \
    "/usr/bin/python" \
    "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control_keyboard/setup.py" \
    build --build-base "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/build/remote_control_keyboard" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/install" --install-scripts="/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/install/bin"
