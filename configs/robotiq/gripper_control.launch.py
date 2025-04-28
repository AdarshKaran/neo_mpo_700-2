# Neobotix GmbH
# Author: Adarsh Karan K P

import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import os

def execution_stage(context: LaunchContext,
                    gripper_type,
                    controller_spawner_timeout,
                    use_mock,
                    controllers_file):

    launch_actions = []

    # Initialize Arguments
    gripper_typ = str(gripper_type.perform(context))
    use_mock = str(use_mock.perform(context)).lower()

    initial_gripper_controller_name = ""
    if gripper_typ:
        if gripper_typ == '2f_140':
            initial_gripper_controller_name = "robotiq_2f_140_gripper_controller"
        elif gripper_typ == '2f_85':
            initial_gripper_controller_name = "robotiq_2f_85_gripper_controller"
        elif gripper_typ == 'epick':
            initial_gripper_controller_name = "epick_gripper_action_controller"

    if gripper_typ == '2f_140' or gripper_typ == '2f_85':
        robotiq_gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                initial_gripper_controller_name,
                "--controller-manager", "/controller_manager",
                "-p", controllers_file,
                "--controller-manager-timeout", controller_spawner_timeout,
            ],
        )

        launch_actions.append(robotiq_gripper_controller_spawner)

        # Activation controller is only needed on real hardware
        if use_mock == 'false':
            robotiq_activation_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robotiq_activation_controller", "-c",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    controller_spawner_timeout
                ]
            )

            launch_actions.append(robotiq_activation_controller_spawner)

    elif gripper_typ == 'epick': #TODO: Test epick gripper and epick_gripper_action_controller missing?
        epick_status_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_gripper_controller_name, "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout
            ]
        )

        epick_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["epick_status_publisher_controller", "-c",
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

    declare_mock_arm_cmd = DeclareLaunchArgument(
        'use_mock', default_value='False',
        description="Mock arm and gripper (if available)"
    )

    declare_controllers_cmd = DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robotiq_description"), "config", "robotiq_controllers.yaml"]
        ),
        description="YAML file with the controllers configuration.",
    )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('controller_spawner_timeout'),
            LaunchConfiguration('use_mock'),
            LaunchConfiguration('controllers_file')
        ])

    ld = LaunchDescription([
        declare_gripper_type_cmd,
        declare_timeout_cmd,
        declare_mock_arm_cmd,
        declare_controllers_cmd,
        opq_function
    ])
    return ld
