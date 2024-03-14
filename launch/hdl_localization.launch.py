from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

pkg_dir = get_package_share_directory("hdl_localization")


def generate_launch_description():
    list = [
        DeclareLaunchArgument("points_topic", default_value="/velodyne_points"),
        DeclareLaunchArgument("odom_child_frame_id", default_value="base_link"),
        DeclareLaunchArgument("use_imu", default_value="false"),
        DeclareLaunchArgument("invert_imu_acc", default_value="false"),
        DeclareLaunchArgument("invert_imu_gyro", default_value="false"),
        DeclareLaunchArgument("use_global_localization", default_value="true"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "enable_robot_odometry_prediction", default_value="false"
        ),
        DeclareLaunchArgument("robot_odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("plot_estimation_errors", default_value="false"),
        DeclareLaunchArgument(
            "map_file", default_value=os.path.join(pkg_dir, "data", "map.pcd")
        ),
        Node(
            package="hdl_localization",
            executable="globalmap_server",
            output="screen",
            parameters=[
                {
                    "globalmap_pcd": LaunchConfiguration("map_file"),
                    "convert_utm_to_local": "true",
                    "downsample_resolution": 0.1,
                }
            ],
        ),
        Node(
            package="hdl_localization",
            executable="hdl_localization",
            output="screen",
            parameters=[
                {
                    "odom_child_frame_id": LaunchConfiguration("odom_child_frame_id"),
                    "use_imu": LaunchConfiguration("use_imu"),
                    "invert_acc": LaunchConfiguration("invert_imu_acc"),
                    "invert_gyro": LaunchConfiguration("invert_imu_gyro"),
                    "cool_time_duration": 2.0,
                    "enable_robot_odometry_prediction": LaunchConfiguration(
                        "enable_robot_odometry_prediction"
                    ),
                    "robot_odom_frame_id": LaunchConfiguration("robot_odom_frame_id"),
                    "reg_method": "NDT_OMP",
                    "ndt_neighbor_search_method": "DIRECT7",
                    "ndt_neighbor_search_radius": 2.0,
                    "ndt_resolution": 1.0,
                    "downsample_resolution": 0.1,
                    "specify_init_pose": True,
                    "init_pos_x": 0.0,
                    "init_pos_y": 0.0,
                    "init_pos_z": 0.0,
                    "init_ori_w": 1.0,
                    "init_ori_x": 0.0,
                    "init_ori_y": 0.0,
                    "init_ori_z": 0.0,
                    "use_global_localization": LaunchConfiguration(
                        "use_global_localization"
                    ),
                }
            ],
            remappings=[
                ("/velodyne_points", LaunchConfiguration("points_topic")),
                ("/gpsimu_driver/imu_data", LaunchConfiguration("imu_topic")),
            ],
        ),
    ]

    return LaunchDescription(list)
