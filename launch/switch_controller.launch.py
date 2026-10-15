import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory('tiago_arm_velocity_controller'),
        'config',
        'tiago_dual_arm_velocity_controllers.yaml'
    )

    # 1. Inform controller_manager of both plugin types
    set_left_type = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set', '/controller_manager',
            'arm_left_velocity_controller.type',
            'tiago_arm_velocity_controller/TiagoArmVelocityPositionController'
        ],
        output='screen'
    )

    set_right_type = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set', '/controller_manager',
            'arm_right_velocity_controller.type',
            'tiago_arm_velocity_controller/TiagoArmVelocityPositionController'
        ],
        output='screen'
    )

    # 2. Spawn both controllers into inactive state
    spawn_left = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_left_velocity_controller', '-c', '/controller_manager', '-p', controller_yaml, '--inactive'],
        output='screen'
    )

    spawn_right = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_right_velocity_controller', '-c', '/controller_manager', '-p', controller_yaml, '--inactive'],
        output='screen'
    )

    # 3. Atomically switch both arms together
    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--controller-manager', '/controller_manager',
            '--activate', 'arm_left_velocity_controller', 'arm_right_velocity_controller',
            '--deactivate', 'arm_left_controller', 'arm_right_controller',
            '--strict',
            '--activate-asap',
        ],
        output='screen'
    )

    # Sequence execution
    right_after_left_param = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=set_left_type, on_exit=[set_right_type])
    )
    spawn_left_after_params = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=set_right_type, on_exit=[spawn_left])
    )
    spawn_right_after_left = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_left, on_exit=[spawn_right])
    )
    switch_after_spawns = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_right, on_exit=[switch_controllers])
    )

    return LaunchDescription([
        set_left_type,
        right_after_left_param,
        spawn_left_after_params,
        spawn_right_after_left,
        switch_after_spawns,
    ])