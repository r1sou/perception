from math import inf
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

import os

def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(param["name"], default_value=param["default_value"])
        for param in parameters
    ]
    
def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():

    root = get_package_share_directory('perception')

    node_params = [
        {"name": "node_name", "default_value": "perception_node"},
        {"name": "log_level", "default_value": "info"},

        {"name": "project_root", "default_value": root},
    
        {"name": "camera_config_path", "default_value": os.path.join(root, 'config', 'camera.json')},
        {"name": "laser_config_path", "default_value": os.path.join(root, 'config', 'laser.json')},
        {"name": "client_config_path", "default_value": os.path.join(root, 'config', 'client.json')},
        {"name": "model_config_path", "default_value": os.path.join(root, 'config', 'model.json')},
        {"name": "record_config_path", "default_value": os.path.join(root, 'config', 'record.json')},

        {"name": "show", "default_value": "False"},
        {"name": "debug", "default_value": "False"},

        {"name": "record", "default_value": "False"},
        {"name": "followme", "default_value": "False"},
        {"name": "recognize", "default_value": "True"}
    ]

    launch = declare_configurable_parameters(node_params)

    launch.append(DeclareLaunchArgument(
        'stereonet_pub_web',
        default_value='True',
        description='stereonet_pub_web, if not, we will disable websocket and codec of stereonet depth'
    ))

    launch.append(Node(
        package='perception',
        executable='perception_node',
        # name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[set_configurable_parameters(node_params)],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    ))

    # 编码节点
    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            # 左图和深度拼接后的图
            'codec_sub_topic': '/image_left_raw',
            'codec_in_format': 'nv12',
            'codec_pub_topic': '/image_jpeg_show',
            'codec_out_format': 'jpeg',
            'log_level': 'warn'
        }.items(),
        condition=IfCondition(LaunchConfiguration('stereonet_pub_web'))
    )
    launch.append(codec_node)

    # web展示节点
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg_show',
            'websocket_only_show_image': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('stereonet_pub_web'))
    )
    launch.append(web_node)

    return LaunchDescription(launch)