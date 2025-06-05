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


# Launch utils are very useful library from isaac ros please find in:
# https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/main/isaac_ros_launch_utils/isaac_ros_launch_utils
from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera

# def load_config():
#     """
#     Load the YAML configuration file.
#     """
#     config_path = os.path.join(
#         get_package_share_directory('zed_wrapper'),
#         'config',
#         'zed_multi_camera.yaml'
#     )
#     with open(config_path, 'r') as file:
#         config = yaml.safe_load(file)
#     return config

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'camera',
        NvbloxCamera.zedx,
        choices=[str(NvbloxCamera.zedx)],
        description='The ZED camera type.',
        cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('nvblox_after_shutdown_map_save_path', '', cli=True)
    args.add_arg('num_cameras')

    args.add_arg('mode', default_value='mapping', description='Launch mode: mapping or navigation')
    args.add_arg('multi_depth', default_value='false', description='Enable multi-depth')
    args.add_arg('nvblox_map_path', default_value='', description='Path to saved Nvblox occupancy map')


    actions = args.get_launch_actions()

    ############# DEPRECATED YAML PARAMS ####################
    # config = load_config()
    # mode = config['zed_multi_camera'].get('mode')  
    # multi_depth = config['zed_multi_camera'].get('multi_depth') 
    # print(mode)
    ############# DEPRECATED YAML PARAMS ####################

    mode = args.mode
    multi_depth = args.multi_depth

    if mode == 'mapping_light':
        nvblox_mode = NvbloxMode.static
        nvblox_map_path = args.nvblox_map_path
        if multi_depth:
            num_cameras = 3
        else:
            num_cameras = 1
            
    elif mode == 'mapping':
        nvblox_mode = NvbloxMode.static
        nvblox_map_path = args.nvblox_map_path
        if multi_depth:
            num_cameras = 3
        else:
            num_cameras = 1

    elif mode == 'meshing':
        nvblox_mode = NvbloxMode.static
        nvblox_map_path = args.nvblox_map_path

        if multi_depth:
            num_cameras = 3
        else:
            num_cameras = 1
            
    elif mode == 'navigation':
        nvblox_mode = NvbloxMode.dynamic
        nvblox_map_path = ''
        if multi_depth:
            num_cameras = 3
        else:
            num_cameras = 1
    else:
        print("Wrong mode selected, try again")


    # Nvblox
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox_zed.launch.py',
            launch_arguments={
                'base_config' : mode,
                'mode': nvblox_mode,
                'camera': args.camera,
                'num_cameras' : num_cameras,
                'after_shutdown_map_save_path': nvblox_map_path
            },
        ))

    return LaunchDescription(actions)
