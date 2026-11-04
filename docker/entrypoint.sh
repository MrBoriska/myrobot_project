#!/usr/bin/env bash
# Container entry point.
#
# Build artefacts live in /opt/ws (a named volume), never in the mounted
# workspace: the host and the container share the source tree but not the build,
# because --symlink-install points into the source tree at absolute paths.
set -e

source /opt/ros/humble/setup.bash

WORKSPACE="${WORKSPACE:-/myrobot_project}"
WS_BUILD="${WS_BUILD:-/opt/ws/build}"
WS_INSTALL="${WS_INSTALL:-/opt/ws/install}"

if [ "${SKIP_BUILD:-0}" != "1" ] && [ -d "${WORKSPACE}/src" ]; then
    cd "${WORKSPACE}"
    # Incremental: a no-op rebuild costs a few seconds, a real one is what you
    # want after editing sources from the host.
    colcon build --symlink-install \
        --build-base "${WS_BUILD}" --install-base "${WS_INSTALL}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release || {
            echo "[entrypoint] build failed; dropping into a shell" >&2
            exec bash
        }
fi

if [ -f "${WS_INSTALL}/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WS_INSTALL}/setup.bash"
fi

exec "$@"
