#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
from pathlib import Path
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import get_xacro_content


def launch_setup(context, *args, **kwargs):
    # robot driver configuration
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
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
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)

    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    # joint_states_remapping = LaunchConfiguration('joint_states_remapping', default=PathJoinSubstitution([hw_ns, 'joint_states']))
    
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

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

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')

    # robot_description
    robot_description = {
        'robot_description': get_xacro_content(
            context,
            xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro', 
            robot_ip=robot_ip,
            report_type=report_type,
            ft_sensor_mode=ft_sensor_mode,
            ft_coord=ft_coord,
            ft_c_axis=ft_c_axis,
            ft_f_ref=ft_f_ref,
            ft_limits=ft_limits,
            ft_kp=ft_kp,
            ft_ki=ft_ki,
            ft_kd=ft_kd,
            ft_xe_limit=ft_xe_limit,
            ft_zero_on_activate=ft_zero_on_activate,
            home_on_activate=home_on_activate,
            home_pose=home_pose,
            home_joints=home_joints,
            home_speed=home_speed,
            home_acc=home_acc,
            home_joint_speed=home_joint_speed,
            home_joint_acc=home_joint_acc,
            tcp_speed=tcp_speed,
            tcp_acc=tcp_acc,
            tcp_radius=tcp_radius,
            cmd_queue_max=cmd_queue_max,
            baud_checkset=baud_checkset,
            default_gripper_baud=default_gripper_baud,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300=model1300,
            robot_sn=robot_sn,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            mesh_suffix=mesh_suffix,
            kinematics_suffix=kinematics_suffix,
            ros2_control_plugin=ros2_control_plugin,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
            add_realsense_d435i=add_realsense_d435i,
            add_d435i_links=add_d435i_links,
            add_other_geometry=add_other_geometry,
            geometry_type=geometry_type,
            geometry_mass=geometry_mass,
            geometry_height=geometry_height,
            geometry_radius=geometry_radius,
            geometry_length=geometry_length,
            geometry_width=geometry_width,
            geometry_mesh_filename=geometry_mesh_filename,
            geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
            geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
            geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
            geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
        )
    }
    robot_description = yaml.dump(robot_description)

    # robot joint state launch
    # xarm_description/launch/_robot_joint_state.launch.py
    robot_joint_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_joint_state.launch.py'])),
        launch_arguments={
            'robot_description': robot_description,
        }.items(),
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': robot_description,
        }.items(),
    )
    
    return [
        # robot_driver_launch,
        robot_joint_state_launch,
        ros2_control_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
