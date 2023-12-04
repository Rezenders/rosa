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
import launch
import launch_pytest
import launch_ros

from pathlib import Path

import pytest

import sys

from ros_pytest.fixture import tester_node


test_node = 'test_executor_integration'
tested_node = 'executor_integration'
rosa_kb_name = 'rosa_kb_executor_integration'


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parents[1]

    executor_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_to_test / 'rosa_execute' /
                'executor_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name=tested_node,
        output='screen',
    )
    path_kb = Path(__file__).parents[2] / 'rosa_kb'
    path_config = path_kb / 'config'
    path_test_data = path_kb / 'test' / 'test_data'

    rosa_kb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_kb / 'rosa_kb' / 'rosa_kb_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name=rosa_kb_name,
        output='screen',
        parameters=[{
            'schema_path': [str(path_config / 'schema.tql')],
            'data_path': [str(path_test_data / 'test_data.tql')],
            'database_name': 'test_' + tested_node
        }]
    )
    return launch.LaunchDescription([
        executor_node,
        rosa_kb_node,
    ])


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_lc_configure(tester_node):
    assert tester_node.lc_configure(tested_node)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_lc_configure_activate(tester_node):
    assert tester_node.lc_configure_activate(tested_node)
