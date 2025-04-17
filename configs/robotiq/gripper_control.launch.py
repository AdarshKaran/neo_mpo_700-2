# Neobotix GmbH
# Author: Adarsh Karan K P

import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

def execution_stage(context: LaunchContext,
                    gripper_type,
                    controller_spawner_timeout):

    launch_actions = []

    # Initialize Arguments
    gripper_typ = str(gripper_type.perform(context))

    if gripper_typ == '2f_140' or gripper_typ == '2f_85':
        robotiq_gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_gripper_controller", "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout
            ]
        )

        robotiq_activation_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_activation_controller", "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout
            ]
        )

        launch_actions.append(robotiq_gripper_controller_spawner)
        launch_actions.append(robotiq_activation_controller_spawner)

    elif gripper_typ == 'epick': #TODO: Test epick gripper
        epick_status_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_status_publisher_controller", "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout
            ]
        )

        epick_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_controller", "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout
            ]
        )

        launch_actions.append(epick_status_controller_spawner)
        launch_actions.append(epick_controller_spawner)

    return launch_actions

def generate_launch_description():

    # Declare the launch arguments
    declare_gripper_type_cmd = DeclareLaunchArgument(
        'gripper_type', default_value='',
        choices=['', '2f_140', '2f_85', 'epick'],
        description='Gripper Types\n\t'
    )

    declare_timeout_cmd = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="300",
        description="Timeout used when spawning controllers.",
    )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('controller_spawner_timeout')
        ])

    ld = LaunchDescription([
        declare_gripper_type_cmd,
        declare_timeout_cmd,
        opq_function
    ])
    return ld
