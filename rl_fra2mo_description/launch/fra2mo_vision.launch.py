from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    marker_size = LaunchConfiguration('marker_size', default='0.1')
    marker_id = LaunchConfiguration('marker_id', default='115')

    fra2mo_explore_launch_path = os.path.join(
        get_package_share_directory('rl_fra2mo_description'), 'launch', 'fra2mo_explore.launch.py'
    )

    aruco_single_launch_path = os.path.join(
        get_package_share_directory('aruco_ros'), 'launch', 'single.launch.py'
    )

    fra2mo_explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fra2mo_explore_launch_path)
    )

    aruco_single_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_single_launch_path),
        launch_arguments={
            'marker_size': marker_size,
            'marker_id': marker_id,
        }.items(),
    )
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '1.57', '3.14', '1.57', 'camera_frame', 'stereo_gazebo_left_camera_optical_frame'],
        output='screen',
    )
    
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
    )

    follow_vis_points_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rl_fra2mo_description',
                executable='follow_vis_points.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        fra2mo_explore_launch,
        aruco_single_launch,
        static_transform_publisher,
        rqt_image_view,
        follow_vis_points_node,
    ])

