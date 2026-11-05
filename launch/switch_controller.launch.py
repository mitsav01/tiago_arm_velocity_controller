import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


CONTROLLER_TYPE = "tiago_velocity_controller_ns/TiagoVelocityController"


def generate_launch_description():

    controller_yaml = os.path.join(
        get_package_share_directory("tiago_arm_velocity_controller"),
        "config",
        "tiago_dual_arm_velocity_controllers.yaml",
    )

    # 1. Register left controller type
    set_left_type = ExecuteProcess(
        cmd=[
            "ros2", "param", "set",
            "/controller_manager",
            "arm_left_velocity_controller.type",
            CONTROLLER_TYPE,
        ],
        output="screen",
    )

    # 2. Register right controller type
    set_right_type = ExecuteProcess(
        cmd=[
            "ros2", "param", "set",
            "/controller_manager",
            "arm_right_velocity_controller.type",
            CONTROLLER_TYPE,
        ],
        output="screen",
    )

    # 3. Spawn left controller inactive
    spawn_left = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_left_velocity_controller",
            "-c", "/controller_manager",
            "-p", controller_yaml,
            "--inactive",
        ],
        output="screen",
    )

    # 4. Spawn right controller inactive
    spawn_right = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_right_velocity_controller",
            "-c", "/controller_manager",
            "-p", controller_yaml,
            "--inactive",
        ],
        output="screen",
    )

    # 5. Switch from position to velocity control
    switch_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "-c", "/controller_manager",
            "--deactivate",
            "arm_left_controller",
            "arm_right_controller",
            "--activate",
            "arm_left_velocity_controller",
            "arm_right_velocity_controller",
            "--strict",
        ],
        output="screen",
    )

    # Sequential startup
    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_left_type,
                on_exit=[set_right_type],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_right_type,
                on_exit=[spawn_left],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_left,
                on_exit=[spawn_right],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_right,
                on_exit=[switch_controllers],
            )
        ),
    ]

    return LaunchDescription(handlers + [set_left_type])