#!/usr/bin/env python3

# Version: 1.0.0
# Purpose: ROS 2 launch configuration for agricultural robotics system
# Dependencies:
# - launch v1.0.0
# - launch_ros v0.19.4

import os
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch.logging import launch_config

# System constants
DEFAULT_NAMESPACE = 'agricultural_robotics'
QOS_RELIABILITY = 'RELIABLE'
QOS_DURABILITY = 'TRANSIENT_LOCAL'
QOS_HISTORY = 'KEEP_LAST'
REAL_TIME_PRIORITY = 95
MAX_DEVICES = 24
RECOVERY_TIMEOUT = 1800  # 30 minutes in seconds

@launch_config
def generate_launch_description() -> LaunchDescription:
    """Generates optimized launch configuration for agricultural robotics system."""
    
    # Launch arguments for system configuration
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=DEFAULT_NAMESPACE,
        description='Top-level namespace for system components'
    )

    # Environment setup for real-time performance
    env_vars = GroupAction([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', 
            '[{severity}] [{time}] [{name}]: {message}'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ])

    # Fleet Controller node configuration
    fleet_controller_node = Node(
        package='agricultural_robotics',
        executable='fleet_controller_node',
        name='fleet_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'update_rate': 10.0,
            'max_devices': MAX_DEVICES,
            'min_battery_threshold': 0.25,
            'real_time_priority': REAL_TIME_PRIORITY,
            'cpu_affinity': '0-3',
            'recovery_timeout': RECOVERY_TIMEOUT,
            'qos_overrides': {
                '/device/status': {
                    'reliability': QOS_RELIABILITY,
                    'durability': QOS_DURABILITY,
                    'history': QOS_HISTORY,
                    'depth': 10
                }
            }
        }],
        remappings=[
            ('fleet/device_status', 'device/status'),
            ('fleet/telemetry', 'telemetry/data'),
            ('fleet/mission_commands', 'mission/commands')
        ]
    )

    # Navigation Controller node configuration
    navigation_controller_node = Node(
        package='agricultural_robotics',
        executable='navigation_controller_node',
        name='navigation_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'path_planning_frequency': 1.0,
            'safety_distance': 5.0,
            'max_velocity': 2.0,
            'real_time_priority': 90,
            'cpu_affinity': '4-7',
            'qos_overrides': {
                '/navigation/path': {
                    'reliability': QOS_RELIABILITY,
                    'durability': QOS_DURABILITY,
                    'history': QOS_HISTORY,
                    'depth': 5
                }
            }
        }]
    )

    # Telemetry Collector node configuration
    telemetry_collector_node = Node(
        package='agricultural_robotics',
        executable='telemetry_collector_node',
        name='telemetry_collector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'publish_rate': 10,
            'batch_size': 100,
            'storage_path': '/data/telemetry',
            'real_time_priority': 85,
            'cpu_affinity': '8-11',
            'qos_overrides': {
                '/telemetry/data': {
                    'reliability': QOS_RELIABILITY,
                    'durability': QOS_DURABILITY,
                    'history': QOS_HISTORY,
                    'depth': 100
                }
            }
        }]
    )

    # System-wide parameter configuration
    system_params = GroupAction([
        SetParameter('use_sim_time', False),
        SetParameter('robot_namespace', LaunchConfiguration('namespace')),
        SetParameter('max_fleet_size', MAX_DEVICES)
    ])

    # Assemble launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(namespace_arg)
    
    # Add environment setup
    ld.add_action(env_vars)
    
    # Add system parameters
    ld.add_action(system_params)
    
    # Add nodes with real-time configurations
    ld.add_action(fleet_controller_node)
    ld.add_action(navigation_controller_node)
    ld.add_action(telemetry_collector_node)
    
    return ld