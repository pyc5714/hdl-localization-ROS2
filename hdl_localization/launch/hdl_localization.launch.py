import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch description object
    ld = LaunchDescription()

    # Set the path to the C++ executable
    package_name_1 = 'hdl_localization'  # Replace with the actual package name
    executable_name_1 = 'hdl_localization_nodelet'  # Replace with the actual executable name
    executable_1 = os.path.join(
        os.getcwd(), 'install', package_name_1, 'lib', package_name_1, executable_name_1
    )


    executable_name_2 = 'globalmap_server_nodelet'  # Replace with the actual executable name
    executable_2 = os.path.join(
        os.getcwd(), 'install', package_name_1, 'lib', package_name_1, executable_name_2
    )

    # package_name_1 = 'hdl_localization'  # Replace with the actual package name
    # executable_name_1 = 'hdl_localization_node'  # Replace with the actual executable name
    # executable_1 = os.path.join(
    #     os.getcwd(), 'install', package_name_1, 'lib', executable_name_1, executable_1
    # )


#   <arg name="use_imu" default="false" />
#   <arg name="invert_imu_acc" default="false" />
#   <arg name="invert_imu_gyro" default="false" />
#   <arg name="use_global_localization" default="true" />
#   <arg name="imu_topic" default="/imu/data" />
#   <arg name="enable_robot_odometry_prediction" value="false" />
#   <arg name="robot_odom_frame_id" value="odom" />
#   <arg name="plot_estimation_errors" value="false" />


    # Set the node options and parameters
    node_options_1 = {
        'use_imu': False,
        'invert_acc': False,
        'invert_gyro': False,
        'use_global_localization': False,
        'robot_odom_frame_id': 'robot_odom',
        'odom_child_frame_id': 'base_link',
        'send_tf_transforms': True,
        'cool_time_duration': 0.5,
        'reg_method': 'NDT_OMP',
        'ndt_neighbor_search_method': 'DIRECT7',
        'ndt_neighbor_search_radius': 2.0,
        'ndt_resolution': 1.0,
        'enable_robot_odometry_prediction': False,
    }


    # <!-- globalmap_server_nodelet -->
    # <node pkg="nodelet" type="nodelet" name="globalmap_server_nodelet" args="load hdl_localization/GlobalmapServerNodelet $(arg nodelet_manager)">
    #   <param name="globalmap_pcd" value="$(find hdl_localization)/data/map.pcd" />
    #   <param name="convert_utm_to_local" value="true" />
    #   <param name="downsample_resolution" value="0.1" />
    # </node>

    node_options_2 = {
        'globalmap_pcd': '/home/ros2_2/src/hdl_localization/data/map.pcd',
        'convert_utm_to_local': True,
        'downsample_resolution': 0.1,   
    }

    # Create the node action
    node_action_1 = Node(
        package=package_name_1,
        executable=executable_1,
        output='screen',
        parameters=[node_options_1],
    )


    node_action_2 = Node(
        package=package_name_1,
        executable=executable_2,
        output='screen',
        parameters=[node_options_2],
    )

    # Add the node action to the launch description
    ld.add_action(node_action_1, node_action_2)

    return ld
