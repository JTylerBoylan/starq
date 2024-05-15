import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
IMU_CONFIG_FILE = os.path.join('/home/nvidia/starq_ws/src/starq/config', 'imu_cv7.yml')
LEFT_CAMERA_CONFIG_PATH = os.path.join('/home/nvidia/starq_ws/src/starq/config', 'left_camera.yaml')
RIGHT_CAMERA_CONFIG_PATH = os.path.join('/home/nvidia/starq_ws/src/starq/config', 'right_camera.yaml')
EKF_CONFIG_PATH = os.path.join('/home/nvidia/starq_ws/src/starq/config', 'ekf.yaml')

def generate_launch_description():
    return LaunchDescription([

        #####################
        ## Microstrain IMU ##
        #####################

        # Microstrain IMU Driver Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
            launch_arguments={
                'configure': 'true',
                'activate': 'true',
                'params_file': IMU_CONFIG_FILE,
                'namespace': '/',
            }.items()
        ),
        # IMU to Robot Base Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link"
                ]
        ),

        #############
        ## Cameras ##
        #############

        # Left Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            parameters=[{
                    'params_file': LEFT_CAMERA_CONFIG_PATH
                }]
        ),
        # Left Camera to Camera Base Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "left_camera_link"
                ]
        ),
        # Right Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            parameters=[{
                    'params_file': RIGHT_CAMERA_CONFIG_PATH
                }]
        ),
        # Right Camera to Camera Base Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "camera_link",
                "--child-frame-id", "right_camera_link"
                ]
        ),
        # Camera Base to Robot Base Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "base_link",
                "--child-frame-id", "camera_link"
                ]
        ),

        ########################
        ## ROBOT LOCALIZATION ##
        ########################

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[EKF_CONFIG_PATH],
        ),

        ###########
        ## STARQ ##
        ###########

        # STARQ Node
        # Node(
        #     package='starq',
        #     executable='starq_node',
        #     output='screen'
        # )
    ])