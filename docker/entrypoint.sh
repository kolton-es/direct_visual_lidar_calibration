#!/bin/bash
# Container entrypoint: provide an X server for the viewers, then run the command.
#
# - DISPLAY set (X11 forwarding from a Linux host): use it.
# - DVLC_VNC=1: start a virtual X server (DVLC_SCREEN, default 2560x1440) with
#   a window manager and serve it over VNC (port 5900) and noVNC in the
#   browser (port 6080, http://localhost:6080/vnc.html).
# - otherwise: start a virtual X server that nobody sees, because the viewers
#   open a (hidden) window even when run headless (calibrate --background).
#
# The viewers render with Mesa's software rasterizer (llvmpipe, OpenGL 4.5),
# so neither mode needs a GPU.
set -e

if [ -z "${DISPLAY:-}" ]; then
    if [ "${DVLC_VNC:-0}" = 1 ]; then
        screen="${DVLC_SCREEN:-2560x1440}x24"
    else
        screen=1920x1080x24
    fi
    Xvfb :99 -screen 0 "$screen" -nolisten tcp >/dev/null 2>&1 &
    export DISPLAY=:99
    for _ in $(seq 1 50); do [ -e /tmp/.X11-unix/X99 ] && break; sleep 0.1; done
    [ -e /tmp/.X11-unix/X99 ] || { echo "entrypoint: Xvfb did not start" >&2; exit 1; }

    if [ "${DVLC_VNC:-0}" = 1 ]; then
        # A window manager, so windows can be moved and resized (the manual
        # initial guess opens an image window next to the 3D viewer).
        openbox >/dev/null 2>&1 &
        x11vnc -display :99 -forever -shared -nopw -quiet -rfbport 5900 >/dev/null 2>&1 &
        websockify --web /usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &
    fi
fi

exec "$@"
