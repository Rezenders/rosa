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
import signal
import sys
from pathlib import Path

import launch
import launch_pytest
import launch_ros

import pytest
from ros_pytest.fixture import tester_node

from metacontrol_execute.executor import Executor

from metacontrol_kb_msgs.msg import Component
from metacontrol_kb_msgs.srv import ComponentQuery
from rcl_interfaces.msg import Parameter

executor_node_name = 'executor_tested'
metacontrol_kb_name = 'metacontrol_kb'


@launch_pytest.fixture
def generate_test_description():
    path_execute_test_data = Path(__file__).parents[0] / 'test_data'
    path_kb = Path(__file__).parents[2] / 'metacontrol_kb'
    path_config = path_kb / 'config'
    path_test_data = path_kb / 'test' / 'test_data'

    metacontrol_kb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_kb / 'metacontrol_kb' / 'metacontrol_kb_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name=metacontrol_kb_name,
        output='screen',
        parameters=[{
            'schema_path': [
                str(path_config / 'schema.tql'),
                str(path_config / 'ros_schema.tql')
            ],
            'data_path': [
                str(path_test_data / 'test_data.tql'),
                str(path_execute_test_data / 'test_data.tql')
            ],
            'database_name': 'test_' + executor_node_name
        }]
    )
    return launch.LaunchDescription([
        metacontrol_kb_node,
    ])


@pytest.mark.usefixtures(fixture=tester_node)
@pytest.fixture()
def executor_node(tester_node):
    executor_node = Executor(executor_node_name)
    tester_node.start_node(executor_node)
    tester_node.activate_lc_node(executor_node_name)
    yield executor_node


def test_start_ros_node(executor_node):
    try:
        node_name = 'executor_mock'
        node_dict = {
            'package': 'metacontrol_execute',
            'executable': 'executor',
            'name': node_name,
            'parameters': [{'test': 'test'}],
        }
        process = executor_node.start_ros_node(node_dict)
        assert process is not False and process.poll() is None and \
            node_name in executor_node.get_node_names()
    finally:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.kill()
        process.wait()


def test_start_ros_launchfile(executor_node):
    try:
        node_dict = {
            'package': 'metacontrol_execute',
            'launch_file': 'executor.launch.py',
            'parameters': [{'test': 'test'}],
        }
        process = executor_node.start_ros_launchfile(node_dict)
        assert process is not False and process.poll() is None and \
            'executor' in executor_node.get_node_names()
    finally:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.kill()
        process.wait()


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_activate_components(executor_node, tester_node):
    try:
        tester_node.activate_lc_node(metacontrol_kb_name)

        component = Component()
        component.name = 'executor_mock'
        component.package = 'metacontrol_execute'
        component.executable = 'executor'

        param = Parameter()
        param.name = 'teste'
        param.value.type = 4
        param.value.string_value = 'teste'

        component.parameters.append(param)

        component2 = Component()
        component2.name = 'executor_mock_2'
        component2.package = 'metacontrol_execute'
        component2.executable = 'executor'
        component2.node_type = 'lifecycle'

        param2 = Parameter()
        param2.name = 'teste'
        param2.value.type = 4
        param2.value.string_value = 'teste'

        component2.parameters.append(param2)

        result = executor_node.activate_components([component, component2])
        component2_state = executor_node.get_lc_node_state(component2.name)

        srv_get = executor_node.create_client(
            ComponentQuery, '/metacontrol_kb/component/active/get')
        component_query = ComponentQuery.Request()
        component_query.component = component
        result_get = executor_node.call_service(srv_get, component_query)

        component_query_2 = ComponentQuery.Request()
        component_query_2.component = component2
        result_get_2 = executor_node.call_service(srv_get, component_query_2)

        assert result is True \
            and component2_state.current_state.id == 3 \
            and component_query.component.is_active is True \
            and component_query_2.component.is_active is True
    finally:
        os.killpg(
            os.getpgid(executor_node.component_pids_dict['executor_mock']),
            signal.SIGKILL)
        os.killpg(
            os.getpgid(executor_node.component_pids_dict['executor_mock_2']),
            signal.SIGKILL)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_set_component_active(executor_node, tester_node):
    tester_node.activate_lc_node(metacontrol_kb_name)

    component = Component()
    component.name = 'executor_mock'
    component.package = 'metacontrol_execute'
    component.executable = 'executor'

    param = Parameter()
    param.name = 'teste'
    param.value.type = 4
    param.value.string_value = 'teste'

    component.parameters.append(param)
    result = executor_node.set_component_active(component, True)

    srv_get = executor_node.create_client(
        ComponentQuery, '/metacontrol_kb/component/active/get')
    component_query = ComponentQuery.Request()
    component_query.component = component
    result_get = executor_node.call_service(srv_get, component_query)

    assert result.success is True and result_get.component.is_active is True
