from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
        .to_moveit_configs()
    )

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "launch", "demo.launch.py")),
        launch_arguments={"ros2_control_hardware_type": "mock_components", "use_rviz": "true"}.items(),
    )

    planner = Node(
        package="path_planning_minimal",
        executable="plan_around_box",
        name="panda_plan_around_box",
        output="screen",
        parameters=[moveit_config.to_dict(), {"interactive": True, "loop_mode": False}],
    )
    return LaunchDescription([demo, planner])

