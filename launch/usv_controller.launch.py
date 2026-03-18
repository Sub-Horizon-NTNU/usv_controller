from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    yaw_kp = LaunchConfiguration("yaw_kp")
    yaw_ki = LaunchConfiguration("yaw_ki")
    yaw_kd = LaunchConfiguration("yaw_kd")
    
    lin_kp = LaunchConfiguration("lin_kp")
    lin_ki = LaunchConfiguration("lin_ki")
    lin_kd = LaunchConfiguration("lin_kd")
    
    max_linear_velocity = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity")
    braking_radius = LaunchConfiguration("braking_radius")

    yaw_kp_arg = DeclareLaunchArgument("yaw_kp", default_value="1.1",description="Yaw angular velocity controller Kp")
    yaw_ki_arg = DeclareLaunchArgument("yaw_ki", default_value="0.4",description="Yaw angular velocity controller Ki")
    yaw_kd_arg = DeclareLaunchArgument("yaw_kd", default_value="0.1",description="Yaw angular velocity controller Kd")
    
    lin_kp_arg = DeclareLaunchArgument("lin_kp", default_value="1.0",description="Linear velocity controller Kp")
    lin_ki_arg = DeclareLaunchArgument("lin_ki", default_value="0.0",description="Linear velocity controller Ki")
    lin_kd_arg = DeclareLaunchArgument("lin_kd", default_value="0.0",description="Linear velocity controller Kd")
    
    max_linear_velocity_arg = DeclareLaunchArgument("max_linear_velocity", default_value="5.0",description="Maximum linear velocity [m/s]")
    max_angular_velocity_arg = DeclareLaunchArgument("max_angular_velocity",default_value="2.0",description="Maximum angular velocity [rad/s]")
    braking_radius_arg = DeclareLaunchArgument("braking_radius", default_value="3.0", description="Distance away from position hold waypoint to start braking [m/s]")

    return LaunchDescription([
        yaw_kp_arg,
        yaw_ki_arg,
        yaw_kd_arg,
        lin_kp_arg,
        lin_ki_arg,
        lin_kd_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        braking_radius_arg,
        Node(
            package="usv_controller",
            executable="usv_controller",
            name="usv_controller",
            output="screen",
            parameters= [
                {"yaw_kp": yaw_kp},
                {"yaw_ki": yaw_ki},
                {"yaw_kd": yaw_kd},
                {"lin_kp": lin_kp},
                {"lin_ki": lin_ki},
                {"lin_kd": lin_kd},
                {"max_linear_velocity": max_linear_velocity},
                {"max_angular_velocity": max_angular_velocity},
                {"braking_radius": braking_radius}
            ])

    ])