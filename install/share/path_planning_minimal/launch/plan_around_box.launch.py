import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load Panda MoveIt config from the correct package for Humble
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda",
                             package_name="moveit_resources_panda_moveit_config")
        .to_moveit_configs()
    )  # usage pattern seen in MoveIt examples/issues. :contentReference[oaicite:5]{index=5}

    # Include the Panda demo to bring up move_group, controllers, and RViz
    panda_demo_launch = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch",
        "demo.launch.py",
    )
    panda_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panda_demo_launch),
        launch_arguments={
            "ros2_control_hardware_type": "mock_components",
            "use_rviz": "true",
        }.items(),
    )  # demo supports setting ros2_control_hardware_type. :contentReference[oaicite:6]{index=6}

    # Your C++ node with the SAME params (URDF, SRDF, kinematics, planners, limits)
    planner = Node(
        package="path_planning_minimal",
        executable="plan_around_box",
        name="panda_plan_around_box",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )  # passing a single dict is the recommended pattern. :contentReference[oaicite:7]{index=7}

    return LaunchDescription([panda_demo, planner])

