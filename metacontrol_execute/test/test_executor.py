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

import pytest
from ros_pytest.fixture import tester_node

from metacontrol_execute.executor import Executor

from metacontrol_kb_msgs.msg import Component
from rcl_interfaces.msg import Parameter

tested_node = 'executor_tested'


@pytest.mark.usefixtures(fixture=tester_node)
@pytest.fixture(scope="module")
def executor_node(tester_node):
    executor_node = Executor(tested_node)
    tester_node.start_node(executor_node)
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
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
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
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.kill()
        process.wait()


def test_activate_components(executor_node):
    try:
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
        assert result is True \
            and component2_state.current_state.id == 3
    finally:
        os.killpg(
            os.getpgid(executor_node.component_pids_dict['executor_mock']),
            signal.SIGTERM)
        os.killpg(
            os.getpgid(executor_node.component_pids_dict['executor_mock_2']),
            signal.SIGTERM)
