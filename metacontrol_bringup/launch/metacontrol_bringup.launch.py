# Copyright 2023 Gustavo Rezende Silva
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
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    schema_path = LaunchConfiguration('schema_path')
    data_path = LaunchConfiguration('data_path')
    database_name = LaunchConfiguration('database_name')
    address = LaunchConfiguration('address')
    force_data = LaunchConfiguration('force_data')
    force_database = LaunchConfiguration('force_database')
    managed_nodes = LaunchConfiguration('managed_nodes')

    pkg_metacontrol_kb = get_package_share_directory(
        'metacontrol_kb')
    pkg_metacontrol_plan = get_package_share_directory(
        'metacontrol_plan')
    pkg_metacontrol_execute = get_package_share_directory(
        'metacontrol_execute')

    metacontrol_kb_launch_path = os.path.join(
        pkg_metacontrol_kb,
        'launch',
        'metacontrol_kb.launch.py')

    metacontrol_plan_launch_path = os.path.join(
        pkg_metacontrol_plan,
        'launch',
        'metacontrol_plan.launch.py')

    metacontrol_execute_launch_path = os.path.join(
        pkg_metacontrol_execute,
        'launch',
        'metacontrol_execute.launch.py')

    default_schema_path = "[{0},{1}]".format(
        os.path.join(pkg_metacontrol_kb, 'config', 'schema.tql'),
        os.path.join(pkg_metacontrol_kb, 'config', 'ros_schema.tql'))

    schema_path_arg = DeclareLaunchArgument(
        'schema_path',
        default_value=default_schema_path,
        description='path for KB schema'
    )

    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value="['']",
        description='path for KB data'
    )

    database_name_arg = DeclareLaunchArgument(
        'database_name',
        default_value='metacontrol_kb',
        description='database name'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='localhost:1729',
        description='typedb server address'
    )

    force_data_arg = DeclareLaunchArgument(
        'force_data',
        default_value='False',
        description='force data'
    )

    force_database_arg = DeclareLaunchArgument(
        'force_database',
        default_value='False',
        description='force database'
    )

    default_managed_nodes = "[{0},{1},{2}]".format(
        'metacontrol_kb', 'configuration_planner', 'executor')

    managed_nodes_arg = DeclareLaunchArgument(
        'managed_nodes',
        default_value=default_managed_nodes,
        description='path for KB schema'
    )

    metacontrol_kb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_kb_launch_path),
        launch_arguments={
            'schema_path': schema_path,
            'data_path': data_path,
            'database_name': database_name,
            'address': address,
            'force_data': force_data,
            'force_database': force_database,
        }.items()
    )

    metacontrol_plan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_plan_launch_path),
    )

    metacontrol_execute = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_execute_launch_path),
    )

    lc_manager_node = Node(
        package='metacontrol_bringup',
        executable='lifecycle_manager',
        parameters=[{
            'managed_nodes': managed_nodes
        }]
    )

    return LaunchDescription([
        schema_path_arg,
        data_path_arg,
        database_name_arg,
        address_arg,
        force_data_arg,
        force_database_arg,
        managed_nodes_arg,
        metacontrol_kb,
        metacontrol_plan,
        metacontrol_execute,
        lc_manager_node,
    ])
