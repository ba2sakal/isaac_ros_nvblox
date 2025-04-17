# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

from launch.substitutions import TextSubstitution
from launch.actions import LogInfo
from launch_ros.actions import Node, ComposableNodeContainer

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nav2_common.launch import RewrittenYaml

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_config():
    """
    Load the YAML configuration file.
    """
    config_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zed_multi_camera.yaml'
    )
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def add_nvblox_dingo_navigation(args: lu.ArgumentContainer) -> List[lut.Action]:
    # Nav2 base parameter file
    actions = []
    
    control = args.control
    if control == 'mppi':
        nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/static_map_nav2_param_mppi_control.yaml')
    elif control == 'shim_mppi':
        nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/nav2_params_shim_mppi.yaml')
    elif control == 'smaclattice':
        nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/nav2_SmacStateLattice_mppi.yaml')
    else:
        nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/nav2_params.yaml')  #DWB Controller
        
    actions.append(lu.log_info(['Load navigation parameters from: ', str(nav_params_path)]))
    actions.append(lut.SetParametersFromFile(str(nav_params_path)))


    actions.append(
    lu.include(
        'nvblox_examples_bringup',
        'launch/perception/occupancy_map_server.launch.py',
        launch_arguments={
            'occupancy_map_yaml_file': args.occupancy_map_yaml_file,
        },
    ))

# # The yaml_filename in navigation_parameters_path somehow overrides the one in occupancy_map_server.launch.py.
#     # We change the parameter in navigation_parameters_path to the input map file.
#     navigation_parameters_path = lu.if_else_substitution(
#         launch_map_server,
#         RewrittenYaml(source_file=navigation_parameters_path,
#                       root_key='',
#                       param_rewrites={'yaml_filename': args.map_yaml_path},
#                       convert_types=True),
#         navigation_parameters_path
#     )
    
    # nvblox specific parameters
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='plugins',
            value=['nvblox_layer', 'inflation_layer'],
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='plugins',
            value=['static_map_layer', 'inflation_layer'],
        ))

    # Modifying nav2 parameters depending on nvblox mode
    mode = NvbloxMode[args.mode]
    
    if mode is NvbloxMode.static:
        costmap_topic_name = '/nvblox_node/static_map_slice'
    elif mode in [NvbloxMode.dynamic, NvbloxMode.people_segmentation]:
        costmap_topic_name = '/nvblox_node/combined_map_slice'
    else:
        raise Exception(f'Navigation in mode {mode} not implemented.')

    print(f'costmap_topic_name: {costmap_topic_name}')

    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))

    actions.append(
        lu.include(
            'nav2_bringup',
            'launch/navigation_launch.py',
            launch_arguments={
                'params_file': str(nav_params_path),
                'use_composition': 'False',
                'container_name': 'navigation_container',
                'use_sim_time': 'False',
            },
        ))

    actions.append(lu.static_transform('map', 'odom'))

    # ROS 2 Component Container
    # container_name = 'navigation_container'
    container_exec='component_container_isolated'
   
    # info = '* Starting Composable node container: /' + container_name
    # actions.append(LogInfo(msg=TextSubstitution(text=info)))
    
    nav2_container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'],
        output='screen',
        # prefix='nice -n -1'  # <-- This line sets the process priority
    )
     
    actions.append(nav2_container)

    return actions

def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
        
    config = load_config()
    nvblox_map_path = config['zed_multi_camera'].get('nvblox_map_path') 
    args.add_arg('occupancy_map_yaml_file', nvblox_map_path + '.yaml', description='Path to occupancy map yaml file')

    args.add_arg('control', 'dwb') # DWB controller will be default if no control is specified while launching
    args.add_arg('mode', 'dynamic')  # 'static' as the default mode
    args.add_arg('container_name', 'navigation_container') 


    args.add_opaque_function(add_nvblox_dingo_navigation)
    return lut.LaunchDescription(args.get_launch_actions())
