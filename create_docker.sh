#!/usr/bin/env bash
# Helper for the development / simulation container.
#
#   ./create_docker.sh build     build the image
#   ./create_docker.sh shell     interactive shell in the workspace
#   ./create_docker.sh sim       synthetic sensors + SLAM (headless, no GPU)
#   ./create_docker.sh check     synthetic sensors + SLAM + accuracy report
#   ./create_docker.sh record    record a rosbag of the synthetic run
#   ./create_docker.sh gazebo    Gazebo server + SLAM (headless, no GPU)
#   ./create_docker.sh gazebo-gui Gazebo + RViz with X11 forwarding
#   ./create_docker.sh rebuild   drop the container build volume and rebuild
#   ./create_docker.sh clean     remove host build/ install/ log/
#
# The workspace is bind-mounted, so everything you change here is visible in the
# container and vice versa.
set -euo pipefail

IMAGE="${IMAGE:-myrobot_project:humble}"
RUN_NAME="${RUN_NAME:-myrobot_project}"
WS_VOLUME="${WS_VOLUME:-myrobot_project_ws}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMAND="${1:-shell}"

build_image() {
    docker build -t "${IMAGE}" -f "${PROJECT_DIR}/Dockerfile" "${PROJECT_DIR}"
}

# Common flags: share the network (DDS discovery between host and container),
# keep the workspace, and pass through X11 when a GUI is requested.
common_flags() {
    echo --rm
    echo --net=host
    echo --ipc=host
    echo -v "${PROJECT_DIR}:/myrobot_project"
    # Container-side build cache, so host and container never share artefacts.
    echo -v "${WS_VOLUME}:/opt/ws"
    echo -w /myrobot_project
}

gui_flags() {
    echo -e "DISPLAY=${DISPLAY}"
    echo -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    echo -v /dev/dri:/dev/dri
}

run() {
    # shellcheck disable=SC2046
    docker run $(common_flags) "$@" </dev/null
}

run_tty() {
    # shellcheck disable=SC2046
    docker run -it $(common_flags) --name "${RUN_NAME}" "$@"
}

case "${COMMAND}" in
    build)
        build_image
        ;;
    shell)
        xhost +local:docker >/dev/null 2>&1 || true
        # shellcheck disable=SC2046
        docker run -it --rm $(common_flags) $(gui_flags) "${IMAGE}" bash
        ;;
    sim)
        run "${IMAGE}" ros2 launch myrobot_custom_slam debug_fake.launch.py
        ;;
    check)
        run "${IMAGE}" bash -lc '
            ros2 launch myrobot_custom_slam debug_fake.launch.py > /tmp/sim.log 2>&1 &
            sleep 25
            ros2 run myrobot_custom_slam check_slam_accuracy.py \
                --settle 2 --duration 30 --verbose
            status=$?
            kill %1 2>/dev/null || true
            tail -5 /tmp/sim.log
            exit $status'
        ;;
    record)
        run "${IMAGE}" bash -lc '
            mkdir -p /tmp/bags
            ros2 bag record -o /tmp/bags/fake_run /scan /imu/data_raw /odom /map \
                /slam/registered_scan /tf /tf_static > /tmp/record.log 2>&1 &
            ros2 launch myrobot_custom_slam debug_fake.launch.py > /tmp/sim.log 2>&1 &
            sleep "${RECORD_SECS:-30}"
            kill %1 %2 2>/dev/null || true
            cp -r /tmp/bags/fake_run /myrobot_project/ 2>/dev/null || true
            echo "bag written to ${PROJECT_DIR}/fake_run"'
        ;;
    gazebo)
        run "${IMAGE}" ros2 launch myrobot_custom_slam slam_gazebo.launch.py \
            gui:=false rviz:=false
        ;;
    gazebo-gui)
        xhost +local:docker >/dev/null 2>&1 || true
        # shellcheck disable=SC2046
        docker run -it --rm $(common_flags) $(gui_flags) "${IMAGE}" \
            ros2 launch myrobot_custom_slam slam_gazebo.launch.py gui:=true rviz:=true
        ;;
    rebuild)
        docker volume rm -f "${WS_VOLUME}" >/dev/null 2>&1 || true
        run "${IMAGE}" bash -lc 'echo container workspace rebuilt'
        ;;
    clean)
        rm -rf "${PROJECT_DIR}/build" "${PROJECT_DIR}/install" "${PROJECT_DIR}/log"
        echo "host build artefacts removed"
        ;;
    *)
        sed -n '2,20p' "${BASH_SOURCE[0]}"
        exit 1
        ;;
esac
