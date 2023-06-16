from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ...

    # nodelet_manager node (ROS1) - Replace with component container node (ROS2)
    component_container_node = Node(
        package='rclcpp_components', executable='component_container',
        name='component_container_node', output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('~/status', '/status'),
        ],
        parameters=[
            {'node_names': ['globalmap_server_nodelet', 'hdl_localization_nodelet']},
        ],
    )

    # globalmap_server component
    globalmap_server_component = ComposableNode(
        package='hdl_localization', plugin='hdl_localization::GlobalmapServerNodelet',
        name='globalmap_server_component',
        remappings=[
            ('/velodyne_points', LaunchConfiguration('points_topic')),
        ],
        parameters=[
            {'globalmap_pcd': os.path.join(LaunchConfiguration('package').perform(), 'data', 'map.pcd')},
            {'convert_utm_to_local': True},
            {'downsample_resolution': 0.1},
        ],
    )

    # hdl_localization component
    hdl_localization_component = ComposableNode(
        package='hdl_localization', plugin='hdl_localization::HdlLocalizationNodelet',
        name='hdl_localization_component',
        remappings=[
            ('/velodyne_points', LaunchConfiguration('points_topic')),
            ('/gpsimu_driver/imu_data', LaunchConfiguration('imu_topic')),
        ],
        parameters=[
            {'odom_child_frame_id': LaunchConfiguration('odom_child_frame_id')},
            {'use_imu': LaunchConfiguration('use_imu')},
            {'invert_acc': LaunchConfiguration('invert_imu_acc')},
            {'invert_gyro': LaunchConfiguration('invert_imu_gyro')},
            {'cool_time_duration': 2.0},
            {'enable_robot_odometry_prediction': LaunchConfiguration('enable_robot_odometry_prediction')},
            {'robot_odom_frame_id': LaunchConfiguration('robot_odom_frame_id')},
            {'reg_method': 'NDT_OMP'},
            {'ndt_neighbor_search_method': 'DIRECT7'},
            {'ndt_neighbor_search_radius': 2.0},
            {'ndt_resolution': 1.0},
            {'downsample_resolution': 0.1},
            {'specify_init_pose': True},
            {'init_pos_x': 0.0},
            {'init_pos_y': 0.0},
            {'init_pos_z': 0.0},
            {'init_ori_w': 1.0},
            {'init_ori_x': 0.0},
            {'init_ori_y': 0.0},
            {'init_ori_z': 0.0},
            {'use_global_localization': LaunchConfiguration('use_global_localization')},
        ],
    )

    # ...

    return LaunchDescription([
        DeclareLaunchArgument('nodelet_manager', default_value='velodyne_nodelet_manager'),
        # ...

        # Replace nodelet_manager node with component_container node
        component_container_node,

        # Replace nodelet nodes with component nodes
        globalmap_server_component,
        hdl_localization_component,

        # ...
    ])

if __name__ == '__main__':
    generate_launch_description()
