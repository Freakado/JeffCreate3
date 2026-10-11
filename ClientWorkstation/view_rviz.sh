#!/usr/bin/env bash

# Source local ROS 2 Jazzy installation
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Ensure workspace overlays are loaded if present
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

export QT_QPA_PLATFORM=xcb
export vblank_mode=0

ROBOT_NS="/JeffCreate3"
TARGET_DOMAIN=10
export ROS_DOMAIN_ID=${TARGET_DOMAIN}

# Configuration Paths
DEFAULT_RVIZ_CONFIG="${HOME}/default.rviz"
PERSPECTIVE_FILE="${HOME}/JeffCreate3-explorer-node.perspective"
RESTAMPER_SCRIPT="${HOME}/qos_republisher.py"
POPUP_SCRIPT="${HOME}/artifact_popup_handler.py"

# Track background process PIDs for guaranteed clean shutdown
BG_PIDS=()

cleanup() {
    echo ""
    echo "Shutting down RViz2, rqt, popup handler, and QoS restamper..."
    # Send SIGINT first for clean rclpy/Qt shutdowns
    for pid in "${BG_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -2 "$pid" 2>/dev/null
        fi
    done
    sleep 0.5
    # Force kill any lingering background jobs
    kill $(jobs -p) 2>/dev/null
}
trap cleanup EXIT SIGINT SIGTERM

# 1. Launch QoS restamper in background
if [ -f "${RESTAMPER_SCRIPT}" ]; then
    echo "Launching QoS restamper from: ${RESTAMPER_SCRIPT}..."
    python3 "${RESTAMPER_SCRIPT}" &
    BG_PIDS+=($!)
else
    echo "Warning: QoS restamper script not found at '${RESTAMPER_SCRIPT}'. Skipping..."
fi

# 2. Launch Artifact Popup Handler in background
if [ -f "${POPUP_SCRIPT}" ]; then
    echo "Launching Artifact Popup Handler from: ${POPUP_SCRIPT}..."
    python3 "${POPUP_SCRIPT}" --ros-args -r __ns:=${ROBOT_NS} &
    BG_PIDS+=($!)
else
    echo "Warning: Popup script not found at '${POPUP_SCRIPT}'. Skipping..."
fi

# 3. Launch rqt with perspective in background
if [ -f "${PERSPECTIVE_FILE}" ]; then
    echo "Launching rqt with perspective: ${PERSPECTIVE_FILE}..."
    rqt --perspective-file "${PERSPECTIVE_FILE}" &
    BG_PIDS+=($!)
else
    echo "Warning: Perspective file '${PERSPECTIVE_FILE}' not found. Starting default rqt..."
    rqt &
    BG_PIDS+=($!)
fi

# RViz config fallback
RVIZ_ARGS=()
if [ -n "$1" ]; then
    RVIZ_ARGS+=("-d" "$1")
elif [ -f "${DEFAULT_RVIZ_CONFIG}" ]; then
    RVIZ_ARGS+=("-d" "${DEFAULT_RVIZ_CONFIG}")
fi

echo "Launching RViz2 for namespace ${ROBOT_NS} on ROS_DOMAIN_ID=${TARGET_DOMAIN}..."

# 4. Launch RViz2 in foreground
ros2 run rviz2 rviz2 "${RVIZ_ARGS[@]}" \
  --ros-args \
  -r __ns:=${ROBOT_NS} \
  -r /tf:=${ROBOT_NS}/tf \
  -r /tf_static:=${ROBOT_NS}/tf_static \
  -r /goal_pose:=${ROBOT_NS}/goal_pose \
  -r /initialpose:=${ROBOT_NS}/initialpose \
  -r /clicked_point:=${ROBOT_NS}/clicked_point