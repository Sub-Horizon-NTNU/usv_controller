import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    default_config = os.path.join(
        get_package_share_directory("usv_controller"), "config", "tuning.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config,
        description="YAML file with tuning parameters")
    initial_heading_arg = DeclareLaunchArgument("initial_heading",default_value="0.0",description="Initial heading when booting up");
    return LaunchDescription([
        config_arg,
        initial_heading_arg,
        Node(
            package="usv_controller",
            executable="usv_controller",
            name="usv_controller",
            output="screen",
            parameters= [
                LaunchConfiguration("config"),
                {"initial_heading": LaunchConfiguration("initial_heading")}
            ])
    ])