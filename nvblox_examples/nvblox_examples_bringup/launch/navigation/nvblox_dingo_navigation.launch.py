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

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def add_nvblox_dingo_navigation(args: lu.ArgumentContainer) -> List[lut.Action]:
    # Nav2 base parameter file
    actions = []
    nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/nav2_params_shim_mppi.yaml')
    actions.append(lut.SetParametersFromFile(str(nav_params_path)))
    #actions.append(lut.SetParameter('use_sim_time', False))
    
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
            value=['nvblox_layer', 'inflation_layer'],
        ))

    # Modifying nav2 parameters depending on nvblox mode
    # mode = NvbloxMode[args.mode]
    mode = NvbloxMode.static
    if mode is NvbloxMode.static:
        costmap_topic_name = '/nvblox_node/static_map_slice'
    elif mode in [NvbloxMode.dynamic, NvbloxMode.people_segmentation]:
        costmap_topic_name = '/nvblox_node/combined_map_slice'
    else:
        raise Exception(f'Navigation in mode {mode} not implemented.')

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
                'container_name': args.container_name,
                'use_composition': 'False',
                'use_sim_time': 'False',
            },
        ))

    actions.append(lu.static_transform('map', 'odom'))

    return actions

def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    full_container_name = "zed_multi" + '/' + "isaac_ros"
    info = 'Loading ZED node in container `' + full_container_name + '`'
    # actions = [LogInfo(msg=info)]
    
    # args.add_arg('mode', default_value='static')  # Hard code the mode as static
    args.add_arg('container_name', full_container_name)   #comment if use_composition is False

    args.add_opaque_function(add_nvblox_dingo_navigation)
    return lut.LaunchDescription(args.get_launch_actions())
