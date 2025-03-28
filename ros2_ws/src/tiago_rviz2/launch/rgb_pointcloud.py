from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate launch description with a single node.

    Node:
        - Node that subscribes to the /gemini2/depth/image_raw and /gemini2/color/image_raw topics,
          and publishes a point cloud with color information to the /gemini2/rgbpoints topic.

    """
    
    return LaunchDescription([
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='point_cloud_xyzrgb',
            remappings=[
                # Remap the depth image topic
                ('depth_registered/image_rect', '/gemini2/depth/image_raw'),
                # Remap the RGB image topic
                ('rgb/image_rect_color', '/gemini2/color/image_raw'),
                # Remap the camera info topic
                ('rgb/camera_info', '/gemini2/color/camera_info'),
                # Remap the output point cloud topic
                ('points', '/gemini2/rgbpoints')
            ]
        )
    ])
