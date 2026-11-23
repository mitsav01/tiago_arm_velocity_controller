#!/usr/bin/env bash

set -Eeuo pipefail

# ==========================================================
# TIAGo Dual Arm Velocity Controller Setup
# ==========================================================

CONTROLLER_MANAGER="/controller_manager"

# Position controllers
LEFT_POSITION_CONTROLLER="arm_left_controller"
RIGHT_POSITION_CONTROLLER="arm_right_controller"

# Velocity controllers
LEFT_VELOCITY_CONTROLLER="arm_left_velocity_controller"
RIGHT_VELOCITY_CONTROLLER="arm_right_velocity_controller"

# Gravity compensation controllers
LEFT_GRAVITY_CONTROLLER="arm_left_gravity_compensation_controller"
RIGHT_GRAVITY_CONTROLLER="arm_right_gravity_compensation_controller"

# Plugin type
VELOCITY_TYPE="tiago_velocity_controller_ns/TiagoVelocityController"

# Configuration
PARAMS_FILE="/home/pal/deployed_ws/src/tiago_arm_velocity_controller/config/tiago_dual_arm_velocity_controllers.yaml"

MAX_RETRIES=5
RETRY_INTERVAL=1

# ==========================================================
# Logging
# ==========================================================

log() {
    echo
    echo "[$(date '+%H:%M:%S')] $1"
}

error() {
    echo
    echo "ERROR: $1" >&2
    exit 1
}

# ==========================================================
# Get controller state
# ==========================================================

get_controller_state() {

    local controller="$1"
    local output
    local line
    local state

    for ((i=1; i<=3; i++)); do

        if ! output=$(ros2 control list_controllers \
            -c "$CONTROLLER_MANAGER" 2>&1); then
            sleep 1
            continue
        fi

        # Remove terminal formatting.
        output=$(printf '%s\n' "$output" |
            sed -E 's/\x1B\[[0-9;]*[[:alpha:]]//g; s/\r$//')

        # Find exact controller row.
        line=$(printf '%s\n' "$output" |
            awk -v name="$controller" '
                $1 == name {
                    print
                    exit
                }
            ')

        if [[ -z "$line" ]]; then
            sleep 1
            continue
        fi

        state=$(printf '%s\n' "$line" | awk '{print $NF}')

        case "$state" in
            active|inactive|unconfigured|finalized)
                echo "$state"
                return 0
                ;;
            *)
                echo "unknown"
                return 0
                ;;
        esac

    done

    # Do not silently treat communication failures as not_loaded.
    if [[ -z "${output:-}" ]]; then
        echo "unknown"
    else
        echo "not_loaded"
    fi
}

# ==========================================================
# Wait for expected controller state
# ==========================================================

wait_for_state() {

    local controller="$1"
    local expected_state="$2"
    local state="unknown"

    for ((i=1; i<=MAX_RETRIES; i++)); do

        sleep "$RETRY_INTERVAL"

        state=$(get_controller_state "$controller")

        echo "Attempt $i/$MAX_RETRIES: $controller = $state"

        if [[ "$state" == "$expected_state" ]]; then
            return 0
        fi

    done

    error "$controller did not reach '$expected_state'. Last state: $state"
}

# ==========================================================
# Check configuration and controller manager
# ==========================================================

check_configuration() {

    log "Checking configuration file"

    [[ -f "$PARAMS_FILE" ]] ||
        error "Configuration file not found: $PARAMS_FILE"

    echo "Configuration file found."
}

check_controller_manager() {

    log "Checking controller manager"

    if ! ros2 control list_controllers \
        -c "$CONTROLLER_MANAGER" >/dev/null 2>&1; then

        error "Cannot communicate with $CONTROLLER_MANAGER"
    fi

    echo "Controller manager is available."
}

# ==========================================================
# Handle position controllers
# ==========================================================

handle_position_controller() {

    local controller="$1"
    local state

    log "Checking $controller"

    state=$(get_controller_state "$controller")

    echo "Current state: $state"

    case "$state" in

        active)

            echo "Deactivating $controller..."

            ros2 control switch_controllers \
                --deactivate "$controller" \
                --strict \
                -c "$CONTROLLER_MANAGER"

            wait_for_state "$controller" "inactive"

            echo "$controller successfully deactivated."

            ;;

        inactive)

            echo "$controller is already inactive."

            ;;

        not_loaded)

            echo "$controller is not loaded. Skipping."

            ;;

        *)

            error "Cannot determine state of $controller: $state"

            ;;

    esac
}

# ==========================================================
# Configure velocity controller
# ==========================================================

configure_velocity_controller() {

    local controller="$1"

    log "Configuring $controller"

    ros2 param set "$CONTROLLER_MANAGER" \
        "${controller}.type" \
        "$VELOCITY_TYPE"

    echo "Controller type configured."
}

# ==========================================================
# Load velocity controller
# ==========================================================

load_velocity_controller() {

    local controller="$1"
    local state

    log "Checking $controller"

    state=$(get_controller_state "$controller")

    echo "Current state: $state"

    case "$state" in

        active)

            echo "$controller is already active."
            ;;

        inactive)

            echo "$controller is already loaded and inactive."
            ;;

        not_loaded)

            echo "$controller is not loaded."

            configure_velocity_controller "$controller"

            log "Loading $controller"

            ros2 control load_controller \
                --set-state inactive \
                "$controller" \
                "$PARAMS_FILE"

            wait_for_state "$controller" "inactive"

            echo "$controller successfully loaded."

            ;;

        unconfigured)

            error "$controller is loaded but unconfigured. Manual investigation required."

            ;;

        finalized)

            error "$controller is finalized. Manual investigation required."

            ;;

        *)

            error "Cannot determine state of $controller."

            ;;

    esac
}

# ==========================================================
# Check gravity compensation controllers
# ==========================================================

check_gravity_controllers() {

    local controller
    local state

    for controller in \
        "$LEFT_GRAVITY_CONTROLLER" \
        "$RIGHT_GRAVITY_CONTROLLER"; do

        state=$(get_controller_state "$controller")

        echo "$controller: $state"

        if [[ "$state" == "active" ]]; then
            error "$controller is active. Check command-interface conflicts before activating velocity controllers."
        fi

        if [[ "$state" == "unknown" ]]; then
            error "Cannot verify $controller state."
        fi

    done
}

# ==========================================================
# Activate both velocity controllers after confirmation
# ==========================================================

optional_activation() {

    local left_state
    local right_state
    local left_position_state
    local right_position_state

    local -a controllers_to_activate=()

    log "Checking velocity controllers before activation"

    left_state=$(get_controller_state "$LEFT_VELOCITY_CONTROLLER")
    right_state=$(get_controller_state "$RIGHT_VELOCITY_CONTROLLER")

    echo "$LEFT_VELOCITY_CONTROLLER: $left_state"
    echo "$RIGHT_VELOCITY_CONTROLLER: $right_state"

    # Verify both controllers are loaded.
    if [[ "$left_state" != "active" && "$left_state" != "inactive" ]]; then
        error "Left velocity controller is not ready."
    fi

    if [[ "$right_state" != "active" && "$right_state" != "inactive" ]]; then
        error "Right velocity controller is not ready."
    fi

    # Add only inactive controllers to the activation request.
    if [[ "$left_state" == "inactive" ]]; then
        controllers_to_activate+=("$LEFT_VELOCITY_CONTROLLER")
    fi

    if [[ "$right_state" == "inactive" ]]; then
        controllers_to_activate+=("$RIGHT_VELOCITY_CONTROLLER")
    fi

    if [[ ${#controllers_to_activate[@]} -eq 0 ]]; then
        echo "Both velocity controllers are already active."
        return 0
    fi

    echo
    echo "=================================================="
    echo "       DUAL ARM VELOCITY CONTROLLER"
    echo "=================================================="

    echo
    echo "Controllers ready for activation:"

    for controller in "${controllers_to_activate[@]}"; do
        echo "  - $controller"
    done

    echo
    echo "Do you want to activate the velocity controllers?"
    echo

    local response

    if ! read -r -p "Activate both arms? [y/N]: " response; then
        response=""
    fi

    case "$response" in

        y|Y|yes|YES)

            log "Performing final safety checks"

            # Verify position controllers are not active.
            left_position_state=$(get_controller_state "$LEFT_POSITION_CONTROLLER")
            right_position_state=$(get_controller_state "$RIGHT_POSITION_CONTROLLER")

            echo "$LEFT_POSITION_CONTROLLER: $left_position_state"
            echo "$RIGHT_POSITION_CONTROLLER: $right_position_state"

            if [[ "$left_position_state" == "active" ||
                  "$right_position_state" == "active" ]]; then

                error "A position controller is active. Refusing velocity controller activation."
            fi

            if [[ "$left_position_state" == "unknown" ||
                  "$right_position_state" == "unknown" ]]; then

                error "Cannot verify position controller states. Refusing activation."
            fi

            # Check gravity compensation controllers.
            check_gravity_controllers

            log "Activating velocity controllers"

            ros2 control switch_controllers \
                --activate "${controllers_to_activate[@]}" \
                --strict \
                -c "$CONTROLLER_MANAGER"

            # Verify activation.
            for controller in "${controllers_to_activate[@]}"; do
                wait_for_state "$controller" "active"
            done

            echo
            echo "SUCCESS: Requested velocity controllers activated."

            ;;

        *)

            echo
            echo "Activation skipped."
            echo "Velocity controllers remain in their current states."

            ;;

    esac
}

# ==========================================================
# Final verification
# ==========================================================

final_verification() {

    log "Final controller status"

    ros2 control list_controllers -c "$CONTROLLER_MANAGER"

    local left_state
    local right_state

    left_state=$(get_controller_state "$LEFT_VELOCITY_CONTROLLER")
    right_state=$(get_controller_state "$RIGHT_VELOCITY_CONTROLLER")

    echo
    echo "=================================================="
    echo "          FINAL VELOCITY CONTROLLER STATUS"
    echo "=================================================="

    echo "Left arm : $left_state"
    echo "Right arm: $right_state"

    if [[ "$left_state" != "active" && "$left_state" != "inactive" ]]; then
        error "Left velocity controller final verification failed."
    fi

    if [[ "$right_state" != "active" && "$right_state" != "inactive" ]]; then
        error "Right velocity controller final verification failed."
    fi

    echo
    echo "Controller setup completed successfully."
}

# ==========================================================
# Main
# ==========================================================

main() {

    echo "=================================================="
    echo "    TIAGo Dual Arm Velocity Controller Setup"
    echo "=================================================="

    check_configuration

    check_controller_manager

    log "Initial controller status"

    ros2 control list_controllers -c "$CONTROLLER_MANAGER"

    # Step 1: Handle both position controllers.
    handle_position_controller "$LEFT_POSITION_CONTROLLER"
    handle_position_controller "$RIGHT_POSITION_CONTROLLER"

    # Step 2: Load both velocity controllers.
    load_velocity_controller "$LEFT_VELOCITY_CONTROLLER"
    load_velocity_controller "$RIGHT_VELOCITY_CONTROLLER"

    # Step 3: Ask before activating either arm.
    optional_activation

    # Step 4: Final verification.
    final_verification

    echo
    echo "=================================================="
    echo "       DUAL ARM SETUP FINISHED"
    echo "=================================================="
}

main "$@"