#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    # 'normal' carries no F/T or CGPIO data at all, so the uf850 defaults to 'rich'.
    report_type = LaunchConfiguration('report_type', default='rich')
    # uf850 force control / homing / Cartesian write params
    ft_sensor_mode = LaunchConfiguration('ft_sensor_mode', default='2')
    ft_coord = LaunchConfiguration('ft_coord', default='1')
    ft_c_axis = LaunchConfiguration('ft_c_axis', default='"0 0 1 0 1 0"')
    ft_f_ref = LaunchConfiguration('ft_f_ref', default='"0 0 8 0 8 0"')
    ft_limits = LaunchConfiguration('ft_limits', default='"0 0 0 0 0 0"')
    ft_kp = LaunchConfiguration('ft_kp', default='"0.005 0.005 0.005 0.005 0.005 0.005"')
    ft_ki = LaunchConfiguration('ft_ki', default='"0.00005 0.00005 0.00005 0.00005 0.00005 0.00005"')
    ft_kd = LaunchConfiguration('ft_kd', default='"0.05 0.05 0.05 0.05 0.05 0.05"')
    ft_xe_limit = LaunchConfiguration('ft_xe_limit', default='"200 200 200 0.1 0.1 0.1"')
    ft_zero_on_activate = LaunchConfiguration('ft_zero_on_activate', default='true')
    home_on_activate = LaunchConfiguration('home_on_activate', default='true')
    home_pose = LaunchConfiguration('home_pose', default='')
    home_joints = LaunchConfiguration('home_joints', default='')
    home_speed = LaunchConfiguration('home_speed', default='100')
    home_acc = LaunchConfiguration('home_acc', default='1000')
    home_joint_speed = LaunchConfiguration('home_joint_speed', default='20')
    home_joint_acc = LaunchConfiguration('home_joint_acc', default='200')
    tcp_speed = LaunchConfiguration('tcp_speed', default='200')
    tcp_acc = LaunchConfiguration('tcp_acc', default='2000')
    tcp_radius = LaunchConfiguration('tcp_radius', default='-1')
    cmd_queue_max = LaunchConfiguration('cmd_queue_max', default='16')
    
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    # robot ros2 control launch
    # xarm_controller/launch/_robot_ros2_control.launch.py
    robot_ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'ft_sensor_mode': ft_sensor_mode,
            'ft_coord': ft_coord,
            'ft_c_axis': ft_c_axis,
            'ft_f_ref': ft_f_ref,
            'ft_limits': ft_limits,
            'ft_kp': ft_kp,
            'ft_ki': ft_ki,
            'ft_kd': ft_kd,
            'ft_xe_limit': ft_xe_limit,
            'ft_zero_on_activate': ft_zero_on_activate,
            'home_on_activate': home_on_activate,
            'home_pose': home_pose,
            'home_joints': home_joints,
            'home_speed': home_speed,
            'home_acc': home_acc,
            'home_joint_speed': home_joint_speed,
            'home_joint_acc': home_joint_acc,
            'tcp_speed': tcp_speed,
            'tcp_acc': tcp_acc,
            'tcp_radius': tcp_radius,
            'cmd_queue_max': cmd_queue_max,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'uf850',
            'add_realsense_d435i': add_realsense_d435i,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }.items(),
    )

    # rviz2 display launch
    # xarm_description/launch/_rviz_display.launch.py
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_rviz_display.launch.py'])),
    )

    # Controller spawners. _ros2_control.launch.py only starts ros2_control_node, so
    # without these nothing claims the interfaces and no controller topics appear.
    #
    # The timeout is generous on purpose: the controller_manager services only come up
    # after the hardware has finished activating, and activation includes a blocking
    # homing move when home_on_activate is set.
    #
    # Controller names are unprefixed. Running with a non-empty prefix needs these
    # names, and sensor_name / frame_id / gpios in uf850_controllers.yaml, adjusted.
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '120',
            ],
        )
        for name in ('joint_state_broadcaster', 'uf_ft_sensor_broadcaster', 'tcp_pose_controller')
    ]

    return LaunchDescription([
        robot_ros2_control_launch,
        rviz2_launch
    ] + controller_spawners)
