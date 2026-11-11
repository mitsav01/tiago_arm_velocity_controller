#!/usr/bin/env bash

set -eo pipefail

# ============================================================
# TIAGo Dual - Custom Velocity Reference Controller Activation
# ROS 2 Humble
# ============================================================

# ROS environment
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/pal/deployed_ws/install/setup.bash"

# Controller manager
CONTROLLER_MANAGER="/controller_manager"

# Existing PAL controllers
PAL_LEFT="arm_left_position_controller"
PAL_RIGHT="arm_right_position_controller"

# Custom controllers
CUSTOM_LEFT="arm_left_velocity_controller"
CUSTOM_RIGHT="arm_right_velocity_controller"

# Custom plugin type
PLUGIN_TYPE="tiago_velocity_controller_ns/TiagoVelocityController"

# ============================================================
# 1. Source ROS environment
# ============================================================

echo "=========================================="
echo " TIAGo Dual Controller Switching"
echo "=========================================="

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS setup file not found: $ROS_SETUP"
    exit 1
fi

source "$ROS_SETUP"

if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
else
    echo "WARNING: Workspace setup not found: $WS_SETUP"
    echo "Make sure the custom controller plugin is already installed."
fi



# ============================================================
# 3. Set custom plugin types
# ============================================================

echo ""
echo "[2/5] Setting custom controller plugin types..."

ros2 param set "$CONTROLLER_MANAGER" \
    "${CUSTOM_LEFT}.type" "$PLUGIN_TYPE"

ros2 param set "$CONTROLLER_MANAGER" \
    "${CUSTOM_RIGHT}.type" "$PLUGIN_TYPE"

echo "Plugin types configured."

# ============================================================
# 4. Spawn custom controllers as inactive
# ============================================================

echo ""
echo "[3/5] Spawning custom controllers (inactive)..."

ros2 run controller_manager spawner \
    "$CUSTOM_LEFT" \
    --inactive \
    -c "$CONTROLLER_MANAGER"

ros2 run controller_manager spawner \
    "$CUSTOM_RIGHT" \
    --inactive \
    -c "$CONTROLLER_MANAGER"

echo "Custom controllers spawned."

# ============================================================
# 5. Switch controllers
# ============================================================

echo ""
echo "[4/5] Switching controllers..."

echo "Deactivating PAL controllers:"
echo "  - $PAL_LEFT"
echo "  - $PAL_RIGHT"

echo "Activating custom controllers:"
echo "  - $CUSTOM_LEFT"
echo "  - $CUSTOM_RIGHT"

ros2 control switch_controllers \
    --deactivate "$PAL_LEFT" "$PAL_RIGHT" \
    --activate "$CUSTOM_LEFT" "$CUSTOM_RIGHT" \
    --strict \
    -c "$CONTROLLER_MANAGER"

echo "Controller switch command completed."

# ============================================================
# 6. Display final controller states
# ============================================================

echo ""
echo "[5/5] Final controller status:"

ros2 control list_controllers -c "$CONTROLLER_MANAGER"

echo ""
echo "=========================================="
echo " Controller switching sequence completed."
echo "=========================================="