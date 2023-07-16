from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,ExecuteProcess

import os

DEFAULT_PATH = os.path.join(os.getcwd()+"/Dataset.csv")


def generate_launch_description():


    file_path_value = LaunchConfiguration('file_path')
 
    file_path_launch_arg = DeclareLaunchArgument(
        'file_path',
        description="Absolute path to the CSV file with data (REQUIRED)"
    )

    change_file_path_conditioned = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            '/data_publisher file ',
            file_path_value
        ]],
        shell=True
    )


    publisher = Node(package="time_diff", executable="data_publisher", name="data_publisher", parameters=[{"file":file_path_value}])

    subscriber = Node(package="time_diff", executable="data_subscriber", name="data_subscriber")

    return LaunchDescription([publisher, subscriber, file_path_launch_arg ,change_file_path_conditioned])


