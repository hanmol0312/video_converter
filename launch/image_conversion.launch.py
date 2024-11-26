import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('image_conv_pkg'),
            'config', 'params_1.yaml'
        ),
        description='Path to the ROS2 parameters file to use.'
    )


    image_server_input_topic_arg = DeclareLaunchArgument(
        'image_server_input_topic',
        default_value='web_cam/image_raw',
        description='Input image topic for the image conversion server.'
    )

    image_server_output_topic_arg = DeclareLaunchArgument(
        'image_server_output_topic',
        default_value='web_cam/converted_image',
        description='Output image topic for the image conversion server.'
    )

    usb_cam_remappings = [
        ('image_raw', LaunchConfiguration('image_server_input_topic')),
    ]

    web_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=usb_cam_remappings
    )

    server_node = Node(
        package='image_conv_pkg',
        executable='image_conversion_server',
        output='screen',
        parameters=[
            {
                'input_image_topic': LaunchConfiguration('image_server_input_topic'),
                'output_image_topic': LaunchConfiguration('image_server_output_topic')
            }
        ]
    )


    camera_group = GroupAction(actions=[web_cam_node, server_node])

    return LaunchDescription([
        params_file_arg,
        image_server_input_topic_arg,
        image_server_output_topic_arg,
        camera_group
    ])
