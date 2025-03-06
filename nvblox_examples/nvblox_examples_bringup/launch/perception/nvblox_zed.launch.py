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
    assert mode is NvbloxMode.static, 'Nvblox only supports static mode for ZED cameras.'
    remappings = []
    remappings.append(('camera_0/depth/image', '/zed_multi/zed_front/depth/depth_registered'))
    remappings.append(('camera_0/depth/camera_info', '/zed_multi/zed_front/depth/camera_info'))
    remappings.append(('camera_0/color/image', '/zed_multi/zed_front/rgb/image_rect_color'))
    remappings.append(('camera_0/color/camera_info', '/zed_multi/zed_front/rgb/camera_info'))
    remappings.append(('pose', '/visual_slam/tracking/vo_pose'))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    mode = NvbloxMode[args.mode]
    camera = NvbloxCamera[args.camera]
    num_cameras = int(args.num_cameras)
    use_lidar = lu.is_true(args.lidar)

    if camera == NvbloxCamera.realsense:
        assert args.num_cameras == 1, 'NvbloxCamera.realsense shall only be set for num_cameras==1'

    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')

    zed_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_zed.yaml')

    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.people_segmentation:
        mode_config = segmentation_config
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode is NvbloxMode.people_detection:
        mode_config = detection_config
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
        assert not use_lidar, 'Can not run lidar with dynamic mode.'
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    if camera is NvbloxCamera.isaac_sim:
        remappings = get_isaac_sim_remappings(mode, num_cameras, use_lidar)
        camera_config = isaac_sim_config
        assert num_cameras <= 1 or mode is not NvbloxMode.people_segmentation, \
            'Can not run multiple cameras with people segmentation in Isaac Sim.'
    elif camera is NvbloxCamera.realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = realsense_config
        assert not use_lidar, 'Can not run lidar for realsense example.'
    elif camera is NvbloxCamera.multi_realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = multi_realsense_config
        assert not use_lidar, 'Can not run lidar for multi realsense example.'
    elif camera in [NvbloxCamera.zed2, NvbloxCamera.zedx]:
        remappings = get_zed_remappings(mode)
        camera_config = zed_config
        assert num_cameras == 1, 'Zed example can only run with 1 camera.'
        assert not use_lidar, 'Can not run lidar for zed example.'
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})
    parameters.append({'use_lidar': use_lidar})

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
             str(mode), "' mode."]))
    return actions


def generate_launch_description() -> LaunchDescription:
    
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('camera')
    args.add_arg('num_cameras', 1)
    args.add_arg('lidar', 'False')

    full_container_name = '/' + "zed_multi" + '/' + "isaac_ros"
    args.add_arg('container_name', full_container_name)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
