import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    nodelet_manager_name = 'hdl_localization_nodelet_manager'

    # declare_nodelet_manager_cmd = DeclareLaunchArgument(
    #     'nodelet_manager',   default_value='hdl_localization_nodelet_manager')
    # declare_points_topic_cmd = DeclareLaunchArgument(
    #     'points_topic',   default_value='/velodyne_points')
    # # input clouds are transformed in odom_child_frame, and then localization is performed in that frame
    # # this is useful to match the LIDAR and IMU coodinate systems
    # declare_odom_child_frame_id_cmd = DeclareLaunchArgument(
    #     'odom_child_frame_id',   default_value='velodyne')
    # declare_use_imu_cmd = DeclareLaunchArgument(
    #     'use_imu',   default_value='false')
    # declare_invert_imu_acc_cmd = DeclareLaunchArgument(
    #     'invert_imu_acc',   default_value='false')
    # declare_invert_imu_gyro_cmd = DeclareLaunchArgument(
    #     'invert_imu_gyro',   default_value='false')
    # declare_use_global_localization_cmd = DeclareLaunchArgument(
    #     'use_global_localization',   default_value='true')
    # declare_imu_topic_cmd = DeclareLaunchArgument(
    #     'imu_topic',   default_value='/imu/data')
    # declare_enable_robot_odometry_prediction_cmd = DeclareLaunchArgument(
    #     'enable_robot_odometry_prediction',   default_value='false')
    # declare_robot_odom_frame_id_cmd = DeclareLaunchArgument(
    #     'robot_odom_frame_id',   default_value='odom')
    # declare_plot_estimation_errors_cmd = DeclareLaunchArgument(
    #     'plot_estimation_errors',   default_value='false')

    # global_localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("hdl_global_localization"), 'launch', 'hdl_global_localization.launch.py')))

    lidar_tf = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0',
                   '0', '1', 'odom', 'velodyne']
    )

    hdl_global_map_server = Node(
        package='hdl_localization',
        # plugin='hdl_localization::GlobalmapServerNodelet',
                executable='hdl_localization_map_server',
                name='hdl_global_map_server',
                parameters=[{
                            'use_sim_time': False,
                            'globalmap_pcd': os.path.join('/home/ROS2_robotnav_taeyong_container/datasets/2022-09-19-10-00-32.pcd'),
                            'convert_utm_to_local': False,
                            'downsample_resolution': 0.1}],
                output="screen"
    )

    hdl_localization = Node(
        package='hdl_localization',
        # plugin='hdl_localization::HdlLocalizationNodelet',
        executable='hdl_localization_node',
        name='hdl_localization',
        parameters=[{
                'use_sim_time': True,
                # odometry frame_id
                'odom_child_frame_id': "velodyne",
                # imu settings
                # during "cool_time", imu inputs are ignored
                'use_imu': True,
                'invert_acc': False,
                'invert_gyro': False,
                'cool_time_duration': 2.0,
                # robot odometry-based prediction
                'enable_robot_odometry_prediction': False,
                'robot_odom_frame_id': "odom",
                # ndt settings
                # available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D
                'reg_method': "NDT_OMP",
                # if NDT is slow for your PC, try DIRECT1 serach method, which is a bit unstable but extremely fast
                'ndt_neighbor_search_method': "DIRECT7",
                'ndt_neighbor_search_radius': 2.0,
                'ndt_resolution': 1.0,
                'downsample_resolution': 0.1,
                # if "specify_init_pose" is true, pose estimator will be initialized with the following params
                # otherwise, you need to input an initial pose with "2D Pose Estimate" on rviz"
                'specify_init_pose': True,
                'init_pos_x': 0.0,
                'init_pos_y': 0.0,
                'init_pos_z': 0.0,
                'init_ori_w': 1.0,
                'init_ori_x': 0.0,
                'init_ori_y': 0.0,
                'init_ori_z': 0.0,
                'use_global_localization': False}],
        # remappings=[('/velodyne_points', '/velodyne_points'), ('/odom',
        #                                               '/hdl_odom'), ('/zed2i/zed_node/imu/data', '/zed2i/zed_node/imu/data')],
        # remappings=[('/velodyne_points', '/points'), ('/odom',
        #                                               '/hdl_odom'), ('/zed2i/zed_node/imu/data', '/imu/data')],
        # prefix=["gdbserver localhost:3000"],
        output="screen",
        # emulate_tty=True
    )

    launch_description = LaunchDescription()

    # launch_description.add_action(global_localization_launch)
    launch_description.add_action(lidar_tf)
    launch_description.add_action(hdl_global_map_server)
    launch_description.add_action(hdl_localization)

    return launch_description
