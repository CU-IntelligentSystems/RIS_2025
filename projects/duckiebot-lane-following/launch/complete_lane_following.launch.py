#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('duckiebot_detection_mapping')
    
    # Paths to files
    urdf_file = os.path.join(pkg_dir, 'urdf', 'duckiebot.urdf')
    zigzag_world = os.path.join(pkg_dir, 'worlds', 'connected_zigzag_track.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_camera_window = LaunchConfiguration('enable_camera_window')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true')
        
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='duckiebot',
        description='Name of the robot')
        
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x position of robot on track')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y position of robot on track')
        
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.1',
        description='z position of robot above ground')
    
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz for visualization')
    
    declare_enable_camera_window_cmd = DeclareLaunchArgument(
        'enable_camera_window',
        default_value='false',
        description='Enable separate camera viewer window')

    # ===== GAZEBO SIMULATION =====
    
    # Start Gazebo server with zigzag world
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': zigzag_world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')))

    # ===== ROBOT SETUP =====
    
    # Robot State Publisher - publishes robot model to TF
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])

    # Joint State Publisher - publishes joint states
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])

    # Spawn the robot in Gazebo (wait 3 seconds for Gazebo to load)
    spawn_entity_cmd = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', robot_name,
                           '-file', urdf_file,
                           '-x', x_pose,
                           '-y', y_pose,
                           '-z', z_pose],
                output='screen')
        ])

    # ===== LANE FOLLOWING SYSTEM =====
    
    # Simple Lane Follower (wait 5 seconds for robot to spawn)
    lane_follower_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='duckiebot_detection_mapping',
                executable='simple_lane_follower.py',
                name='simple_lane_follower',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ])
        ])

    # ===== OPTIONAL COMPONENTS =====
    
    # Camera Viewer (separate window) - optional
    camera_viewer_cmd = Node(
        condition=IfCondition(enable_camera_window),
        package='duckiebot_detection_mapping',
        executable='camera_viewer.py',
        name='camera_viewer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_topic': '/duckiebot/camera/image_raw'}
        ])

    # RViz for visualization - optional but recommended
    rviz_cmd = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'lane_following.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ===== MONITORING AND DEBUG =====
    
    # Lane Detection Monitor (shows detection statistics)
    lane_monitor_cmd = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='duckiebot_detection_mapping',
                executable='duckiebot_mapper.py',
                name='lane_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}])
        ])

    # ===== HELPFUL INFO NODE =====
    
    # Info node that prints helpful instructions
    info_node_cmd = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='duckiebot_detection_mapping',
                executable='info_helper.py',
                name='info_helper',
                output='screen')
        ])

    # ===== BUILD LAUNCH DESCRIPTION =====
    
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_enable_camera_window_cmd)

    # Add simulation components
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    # Add robot components
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    
    # Add lane following system
    ld.add_action(lane_follower_cmd)
    
    # Add optional components
    ld.add_action(camera_viewer_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(lane_monitor_cmd)
    
    # Add info helper (comment out if not created)
    # ld.add_action(info_node_cmd)

    return ld