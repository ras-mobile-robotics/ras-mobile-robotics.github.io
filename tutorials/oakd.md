---
layout: default
title: "OakD Camera Topics"
parent: Tutorials
sort: 14
---

We are changing the oakd config to publish compressed data from the robot. These topics work at a higher rate than uncompressed topics. We then subscribe to the compressed topics on your computer, decompress them and pass it to the RGBD point cloud generator.

# Instructions
1. Update your [OAKD Config](#1-oakd-config)
2. For reference, check if your oakd.launch file looks similar to the one [here](#2-oakdlaunch).
3. Run the Camera launch file and check if the **compressed** topics are publishing data now. Check the topics' frequencies as well. (ros2 topic hz)
4. Now that you have the compressed topics working, use the [RGBD Launch File](#3-rgbd-launch-file) to decompress the topics and run the `point_cloud_xyzrgb_node` node to get the pointcloud topic.
> Note: the bag recorder/player and throttle nodes are commented out. You may use it if you want.


## 1. OAKD Config
Output compressed RGB and Stereo both at 1280 x 720. 

```yaml
oakd:
  ros__parameters:
    camera:
      i_enable_imu: false
      i_enable_ir: false
      i_nn_type: none
      i_pipeline_type: RGBD
      i_usb_speed: SUPER_PLUS
    rgb:
      i_board_socket_id: 0
      i_fps: 30.0
      i_interleaved: false
      i_max_q_size: 10
      i_preview_size: 250
      i_enable_preview: false
      i_low_bandwidth: true
      i_keep_preview_aspect_ratio: true
      i_publish_topic: true
      i_resolution: '1080P'  # Match what the sensor wants
      i_width: 1280
      i_height: 720
    stereo:
      i_publish_topic: true
      i_align_depth: true
      i_subpixel: true
      i_output_point_cloud: true
    pointcloud:
      i_enable_feature: true
      i_publish_topic: true
    use_sim_time: false
```

## 2. oakd.launch

```python
#!/usr/bin/env python3
# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')

    camera = LaunchConfiguration('camera')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    ARGUMENTS = [
        DeclareLaunchArgument('camera', default_value='oakd_pro'),
        DeclareLaunchArgument('params_file',
                              default_value=[PathJoinSubstitution(
                                [pkg_turtlebot4_bringup, 'config', camera]), '.yaml']),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace')
    ]

    namespaced_param_file = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    node = ComposableNodeContainer(
            name='oakd_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='depthai_ros_driver',
                        plugin='depthai_ros_driver::Camera',
                        name='oakd',
                        parameters=[namespaced_param_file]
                    ),
            ],
            output='screen',
        )

    actions = [
        PushRosNamespace(namespace),
        node
    ]
    oakd = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(oakd)
    return ld
```


## 3. RGBD Launch File

```python
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Generate a default timestamped name
    default_output_name = 'processed_points_' + datetime.now().strftime('%Y_%m_%d-%H_%M_%S')

    # 1. Arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/bags/rosbag2_2026_02_27-16_25_39'),
        description='Path to input bag'
    )
    play_rate_arg = DeclareLaunchArgument(
        'rate', default_value='1.0', description='Playback speed'
    )
    # NEW: Output Bag Name Argument
    output_bag_arg = DeclareLaunchArgument(
        'out_bag',
        default_value=default_output_name,
        description='Name/Path of the output recorded bag'
    )

    # 2. Processing Nodes
    republish_rgb = Node(
        package='image_transport', executable='republish', name='republish_rgb',
        parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
        remappings=[('in/compressed', '/robot_10/oakd/rgb/image_raw/compressed'),
                    ('out', '/robot_10/oakd/rgb/image_raw_decompressed')]
    )

    republish_depth = Node(
        package='image_transport', executable='republish', name='republish_depth',
        parameters=[{'in_transport': 'compressedDepth', 'out_transport': 'raw'}],
        remappings=[('in/compressedDepth', '/robot_10/oakd/stereo/image_raw/compressedDepth'),
                    ('out', '/robot_10/oakd/stereo/image_raw_decompressed')]
    )

    point_cloud_node = Node(
        package='depth_image_proc', executable='point_cloud_xyzrgb_node',
        parameters=[{'use_sim_time': True, 'approximate_sync': True, 'slop': 0.2, 'queue_size': 100}],
        remappings=[('rgb/image_rect_color', '/robot_10/oakd/rgb/image_raw_decompressed'),
                    ('rgb/camera_info', '/robot_10/oakd/rgb/camera_info'),
                    ('depth_registered/image_rect', '/robot_10/oakd/stereo/image_raw_decompressed'),
                    ('points', '/robot_10/oakd/points_unthrottled')]
    )

    throttle_pcl = Node(
        package='topic_tools',
        executable='throttle',
        name='pcl_throttler',
        # ADD THIS LINE:
        arguments=['messages', '/robot_10/oakd/points_unthrottled', '2.0', '/oakd/points'],
        parameters=[{'use_sim_time': True}],
    )

    throttle_rgb_compressed = Node(
        package='topic_tools',
        executable='throttle',
        name='rgb_throttler',
        # ADD THIS LINE:
        arguments=['messages', '/robot_10/oakd/rgb/image_raw/compressed', '2.0', '/oakd/rgb/image_raw/compressed'],
        parameters=[{'use_sim_time': True}],
    )

    # 3. Bag Recorder (Now uses LaunchConfiguration)
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 
            '/oakd/points', 
            '/oakd/rgb/image_raw/compressed', 
            '-s', 'mcap', # Use the MCAP storage format
            '-o', LaunchConfiguration('out_bag')],
        output='screen'
    )

    # 4. Bag Player
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--clock', '--rate', LaunchConfiguration('rate')],
        output='screen'
    )

    # 5. Event Handler: Shutdown
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[EmitEvent(event=Shutdown(reason='Bag playback finished'))]
        )
    )

    return LaunchDescription([
        bag_path_arg,
        play_rate_arg,
        output_bag_arg, # Add the new argument here
        republish_rgb,
        republish_depth,
        point_cloud_node,
        # throttle_pcl,
        # throttle_rgb_compressed
        # shutdown_handler
        # TimerAction(period=2.0, actions=[bag_record]),
        # TimerAction(period=3.0, actions=[bag_play])
    ])
```


## 4. lite.launch

> Note: This is for your reference.


```python
#!/usr/bin/env python3
# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction  # noqa: E501
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    diagnostics_enable = EnvironmentVariable('TURTLEBOT4_DIAGNOSTICS', default_value='1')
    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')

    param_file_cmd = DeclareLaunchArgument(                                                                    14:09:16 [45/189]
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='Turtlebot4 Robot param file'
    )

    param_file = LaunchConfiguration('param_file')

    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    # Launch files
    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    joy_teleop_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'joy_teleop.launch.py'])
    diagnostics_launch_file = PathJoinSubstitution(                                                            14:09:16 [25/189]
        [pkg_turtlebot4_diagnostics, 'launch', 'diagnostics.launch.py'])
    rplidar_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'rplidar.launch.py'])
    oakd_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'oakd.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

    actions = [
            PushRosNamespace(namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
                launch_arguments=[('model', 'lite'),
                                  ('param_file', namespaced_param_file)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([joy_teleop_launch_file]),
                launch_arguments=[('namespace', namespace)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rplidar_launch_file])),

            # Delay the OAK-D startup for a bit
            # This prevents spiking the current on the USB by having the lidar and camera
            # start up at the same time as everything else
            TimerAction(
                period=30.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([oakd_launch_file]),
                        launch_arguments=[
                            ('camera', 'oakd_lite'),
                            ('namespace', namespace),
                        ])
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([description_launch_file]),
                launch_arguments=[('model', 'lite')]),
        ]

    if (diagnostics_enable.perform(lc)) == '1':
        actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([diagnostics_launch_file]),
                launch_arguments=[('namespace', namespace)]))

    turtlebot4_lite = GroupAction(actions)

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)
    ld.add_action(turtlebot4_lite)
    return ld
```