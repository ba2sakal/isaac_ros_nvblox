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

from typing import List, Tuple

from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
# from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def get_zed_remappings(mode: NvbloxMode) -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('camera_0/depth/image', '/zed_multi/zed_front/depth/depth_registered'))
    remappings.append(('camera_0/depth/camera_info', '/zed_multi/zed_front/depth/camera_info'))
    remappings.append(('camera_0/color/image', '/zed_multi/zed_front/rgb/image_rect_color'))
    remappings.append(('camera_0/color/camera_info', '/zed_multi/zed_front/rgb/camera_info'))


    remappings.append(('camera_1/depth/image', '/zed_multi/zed_left/depth/depth_registered'))
    remappings.append(('camera_1/depth/camera_info', '/zed_multi/zed_left/depth/camera_info'))
    remappings.append(('camera_1/color/image', '/zed_multi/zed_left/rgb/image_rect_color'))
    remappings.append(('camera_1/color/camera_info', '/zed_multi/zed_left/rgb/camera_info'))


    remappings.append(('camera_2/depth/image', '/zed_multi/zed_right/depth/depth_registered'))
    remappings.append(('camera_2/depth/camera_info', '/zed_multi/zed_right/depth/camera_info'))
    remappings.append(('camera_2/color/image', '/zed_multi/zed_right/rgb/image_rect_color'))
    remappings.append(('camera_2/color/camera_info', '/zed_multi/zed_right/rgb/camera_info'))

    remappings.append(('pose', '/visual_slam/tracking/vo_pose'))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    mode = NvbloxMode[args.mode]
    camera = NvbloxCamera[args.camera]
    num_cameras = int(args.num_cameras)

    config = args.base_config
    if config == 'default':
        base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_default.yaml')
    elif config == 'mapping':
        base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_global_mapping.yaml')
    elif config == 'navigation':
        base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_navigation.yaml')
    else:
        raise Exception(f"Invalid base_config argument: {args.base_config}")


    zed_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_zed.yaml')
    
    dynamics_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_dynamics.yaml')

    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    if camera is NvbloxCamera.isaac_sim:
        remappings = get_isaac_sim_remappings(mode, num_cameras, use_lidar)
        camera_config = isaac_sim_config
    elif camera in [NvbloxCamera.zedx]:
        remappings = get_zed_remappings(mode)
        camera_config = zed_config
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})

    # Add the nvblox node.c
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    # if args.run_standalone:
    #     actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(
            ["Starting nvblox with the '",
             str(camera), "' camera in '",
             str(mode), "' mode with ",
             str(config), " 'base_config'."]))
    return actions


def generate_launch_description() -> LaunchDescription:
    
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('camera')
    args.add_arg('num_cameras', 1)

    args.add_arg('base_config', 'default')

    full_container_name = '/' + "zed_multi" + '/' + "isaac_ros"
    args.add_arg('container_name', full_container_name)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
