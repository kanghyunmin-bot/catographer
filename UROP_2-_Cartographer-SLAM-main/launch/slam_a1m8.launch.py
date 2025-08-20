#!/usr/bin/env python3
import os, tempfile, yaml, textwrap
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _make_runtime_lua(tmpdir: str, base_link: str):
    lua = textwrap.dedent(f"""
    include "map_builder.lua"
    include "trajectory_builder.lua"

    options = {{
      map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER,

      -- frames
      map_frame = "map",
      tracking_frame = "{base_link}",
      published_frame = "{base_link}",
      odom_frame = "odom",
      provide_odom_frame = true,
      publish_frame_projected_to_2d = true,

      -- sensors (2D Lidar 1개)
      num_laser_scans = 1,
      num_multi_echo_laser_scans = 0,
      num_point_clouds = 0,
      num_subdivisions_per_laser_scan = 1,
      use_odometry = false,
      use_nav_sat = false,
      use_landmarks = false,

      -- ★ Cartographer ROS가 반드시 찾는 키들
      lookup_transform_timeout_sec = 0.2,
      submap_publish_period_sec    = 0.3,
      pose_publish_period_sec      = 0.005,
      trajectory_publish_period_sec= 0.03,
      rangefinder_sampling_ratio   = 1.0,
      odometry_sampling_ratio      = 1.0,
      fixed_frame_pose_sampling_ratio = 1.0,
      imu_sampling_ratio           = 1.0,
      landmarks_sampling_ratio     = 1.0,
    }}

    MAP_BUILDER.use_trajectory_builder_2d = true

    -- 2D builder 튜닝 (A1M8 실내 기준)
    TRAJECTORY_BUILDER_2D.min_range = 0.15
    TRAJECTORY_BUILDER_2D.max_range = 12.0
    TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.0
    TRAJECTORY_BUILDER_2D.use_imu_data = false
    TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

    -- 실시간 매칭 권장(원하면 끌 수 있음)
    TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.10

    TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
    TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
    TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.10

    POSE_GRAPH.optimization_problem.huber_scale = 1e1
    POSE_GRAPH.optimize_every_n_nodes = 90

    return options
    """)
    path = os.path.join(tmpdir, "a1m8_2d.lua")
    with open(path, "w") as f:
        f.write(lua)
    return path


def _launch_setup(context, *args, **kwargs):
    # 인자/파일 경로
    rplidar_yaml = LaunchConfiguration('rplidar_yaml').perform(context)
    grid_yaml = LaunchConfiguration('grid_yaml').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    laser_frame = LaunchConfiguration('laser_frame').perform(context)

    # occupancy_grid 파라미터(YAML) 읽어 CLI 인자 변환
    with open(grid_yaml, 'r') as f:
        grid_cfg = yaml.safe_load(f)
    res = str(grid_cfg.get('occupancy_grid', {}).get('resolution', 0.05))
    pub = str(grid_cfg.get('occupancy_grid', {}).get('publish_period_sec', 0.5))

    # 런타임 Lua 생성
    tmpdir = tempfile.mkdtemp(prefix="carto_cfg_")
    lua_path = _make_runtime_lua(tmpdir, base_frame)

    nodes = []

    # RPLIDAR 드라이버
    nodes.append(Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        parameters=[rplidar_yaml],
        output='screen'
    ))

    # base_link↔laser 정적 TF (필요시 x y z yaw pitch roll 수정)
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base',
        arguments=['0','0','0','0','0','0', base_frame, laser_frame],
        output='screen'
    ))

    # Cartographer SLAM 노드 (Lua 임시파일 사용)
    nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer',
        arguments=['-configuration_directory', tmpdir,
                   '-configuration_basename', os.path.basename(lua_path)],
        remappings=[('scan', '/scan')],
        output='screen'
    ))

    # Occupancy Grid 발행 노드 (해상도/주기를 CLI 인자로)
    nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid',
        arguments=['-resolution', res, '-publish_period_sec', pub],
        output='screen'
    ))
    # RViz2 visualization (optional)
    rviz_cfg = os.path.join(
        get_package_share_directory('a1m8_cartographer'),
        'rviz',
        'a1m8_cartographer.rviz')
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))


    return nodes

def generate_launch_description():
    share_dir = get_package_share_directory('a1m8_cartographer')
    rplidar_default = os.path.join(share_dir, 'params', 'rplidar_a1m8.yaml')
    grid_default = os.path.join(share_dir, 'params', 'occupancy_grid.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rplidar_yaml', default_value=rplidar_default),
        DeclareLaunchArgument('grid_yaml', default_value=grid_default),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup)
    ])

