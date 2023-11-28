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
import rclpy
from threading import Thread
from pathlib import Path

import launch
import launch_pytest
import launch_ros

import pytest
from ros_pytest.fixture import tester_node

from metacontrol_execute.executor import Executor

from metacontrol_kb_msgs.msg import Component
from metacontrol_kb_msgs.msg import ComponentConfig
from metacontrol_kb_msgs.msg import ReconfigurationPlan
from metacontrol_kb_msgs.srv import ComponentQuery
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters

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
            'schema_path': str([
                str(path_config / 'schema.tql'),
                str(path_config / 'ros_schema.tql')
            ]),
            'data_path': str([
                str(path_test_data / 'test_data.tql'),
                str(path_test_data / 'ros_test_data.tql'),
                str(path_execute_test_data / 'test_data.tql')
            ]),
            'database_name': 'test_' + executor_node_name
        }]
    )
    return launch.LaunchDescription([
        metacontrol_kb_node,
    ])


def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


@pytest.mark.usefixtures(fixture=tester_node)
@pytest.fixture(scope='module')
def executor_node(tester_node):
    executor_node = Executor(executor_node_name)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(executor_node)
    new_thread = Thread(target=spin_srv, args=(executor, ), daemon=True)
    new_thread.start()

    tester_node.activate_lc_node(executor_node_name)
    yield executor_node
    executor_node.destroy_node()
    executor.shutdown()
    new_thread.join()


def test_start_ros_node(executor_node):
    try:
        node_name = 'executor_mock_start_ros_node'
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
        pgid = os.getpgid(process.pid)
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


def test_start_ros_launchfile(executor_node):
    try:
        node_dict = {
            'package': 'metacontrol_execute',
            'launch_file': 'metacontrol_execute.launch.py',
            'parameters': [{'test': 'test'}],
        }
        process = executor_node.start_ros_launchfile(node_dict)
        assert process is not False and process.poll() is None and \
            'executor' in executor_node.get_node_names()
    finally:
        pgid = os.getpgid(process.pid)
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


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
        component2.node_type = 'LifeCycleNode'

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
            and result_get.component.is_active is True \
            and result_get_2.component.is_active is True
    finally:
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


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


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_deactivate_components(executor_node, tester_node):
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
        component2.node_type = 'LifeCycleNode'

        param2 = Parameter()
        param2.name = 'teste'
        param2.value.type = 4
        param2.value.string_value = 'teste'

        component2.parameters.append(param2)

        result_activate = executor_node.activate_components(
            [component, component2])
        result_deactivate = executor_node.deactivate_components(
            [component, component2])

        component2_state = executor_node.get_lc_node_state(component2.name)

        srv_get = executor_node.create_client(
            ComponentQuery, '/metacontrol_kb/component/active/get')

        component_query = ComponentQuery.Request()
        component_query.component = component
        result_get = executor_node.call_service(srv_get, component_query)

        component_query_2 = ComponentQuery.Request()
        component_query_2.component = component2
        result_get_2 = executor_node.call_service(srv_get, component_query_2)

        assert result_activate is True and result_deactivate is True \
            and component2_state.current_state.id == 2 \
            and result_get.component.is_active is False \
            and result_get_2.component.is_active is False \
            and 'executor_mock' not in executor_node.component_pids_dict
    finally:
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_perform_parameter_adaptation(executor_node, tester_node):
    try:
        tester_node.activate_lc_node(metacontrol_kb_name)

        component = Component()
        component.name = 'ros_typedb_test'
        component.package = 'ros_typedb'
        component.executable = 'ros_typedb'
        component.node_type = 'LifeCycleNode'

        component2 = Component()
        component2.name = 'ros_typedb_test_2'
        component2.package = 'ros_typedb'
        component2.executable = 'ros_typedb'
        component2.node_type = 'LifeCycleNode'

        result = executor_node.activate_components([component, component2])

        config = ComponentConfig(name='ros_typedb_config')
        config_2 = ComponentConfig(name='ros_typedb_config_2')
        result = executor_node.perform_parameter_adaptation([config, config_2])

        get_param_srv = executor_node.create_client(
            GetParameters, '/ros_typedb_test/get_parameters')
        get_param_srv_2 = executor_node.create_client(
            GetParameters, '/ros_typedb_test_2/get_parameters')

        param_req = GetParameters.Request(
            names=[
                'database_name', 'force_database', 'force_data', 'data_path'])
        params = executor_node.call_service(get_param_srv, param_req)
        params_2 = executor_node.call_service(get_param_srv_2, param_req)

        expected_params = [
            ParameterValue(type=4, string_value='ros_typedb_executor'),
            ParameterValue(type=1, bool_value=True),
            ParameterValue(type=1, bool_value=False),
            ParameterValue(
                type=4, string_value="[test_data/test_data.tql]"),
        ]
        expected_params_2 = [
            ParameterValue(type=4, string_value='ros_typedb_executor_2'),
            ParameterValue(type=1, bool_value=False),
            ParameterValue(type=1, bool_value=True),
            ParameterValue(
                type=4, string_value="[test_data/test_data.tql]"),
        ]

        assert result is True and \
            all(p in params.values for p in expected_params) and \
            all(p in params_2.values for p in expected_params_2)
    finally:
        pgid = os.getpgid(executor_node.component_pids_dict['ros_typedb_test'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(
            executor_node.component_pids_dict['ros_typedb_test_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_perform_reconfiguration_plan(executor_node, tester_node):
    try:
        tester_node.activate_lc_node(metacontrol_kb_name)

        component = Component()
        component.name = 'executor_mock'
        component.package = 'metacontrol_execute'
        component.executable = 'executor'
        component.node_type = 'LifeCycleNode'

        component2 = Component()
        component2.name = 'executor_mock_2'
        component2.package = 'metacontrol_execute'
        component2.executable = 'executor'
        component2.node_type = 'LifeCycleNode'

        executor_node.activate_components([component, component2])

        component3 = Component()
        component3.name = 'ros_typedb_test'
        component3.package = 'ros_typedb'
        component3.executable = 'ros_typedb'
        component3.node_type = 'LifeCycleNode'

        component4 = Component()
        component4.name = 'ros_typedb_test_2'
        component4.package = 'ros_typedb'
        component4.executable = 'ros_typedb'
        component4.node_type = 'LifeCycleNode'

        config = ComponentConfig(name='ros_typedb_config')
        config_2 = ComponentConfig(name='ros_typedb_config_2')

        reconfig_plan = ReconfigurationPlan()
        reconfig_plan.components_deactivate = [component, component2]
        reconfig_plan.components_activate = [component3, component4]
        reconfig_plan.component_configurations = [config, config_2]

        result = executor_node.perform_reconfiguration_plan(reconfig_plan)

        assert result is True
    finally:
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(
            executor_node.component_pids_dict['executor_mock_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(executor_node.component_pids_dict['ros_typedb_test'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(
            executor_node.component_pids_dict['ros_typedb_test_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_execute(executor_node, tester_node):
    try:
        tester_node.activate_lc_node(metacontrol_kb_name)

        component = Component()
        component.name = 'executor_mock_2'
        component.package = 'metacontrol_execute'
        component.executable = 'executor'
        component.node_type = 'LifeCycleNode'

        executor_node.activate_components([component])

        executor_node.execute()

        component_state = executor_node.get_lc_node_state(component.name)
        active_nodes = executor_node.get_node_names()

        get_param_srv = executor_node.create_client(
            GetParameters, '/ros_typedb_test_execute/get_parameters')
        param_req = GetParameters.Request(
            names=[
                'database_name', 'force_database', 'force_data', 'data_path'])
        params = executor_node.call_service(get_param_srv, param_req)
        expected_params = [
            ParameterValue(type=4, string_value='ros_typedb_executor'),
            ParameterValue(type=1, bool_value=True),
            ParameterValue(type=1, bool_value=False),
            ParameterValue(
                type=4, string_value="[test_data/test_data.tql]"),
        ]
        assert component_state.current_state.id == 2 and \
            'executor_mock' in active_nodes and \
            'ros_typedb_test_execute' in active_nodes and \
            all(p in params.values for p in expected_params)
    finally:
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(executor_node.component_pids_dict['executor_mock_2'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
        pgid = os.getpgid(
            executor_node.component_pids_dict['ros_typedb_test_execute'])
        os.killpg(pgid, signal.SIGTERM)
        os.waitid(os.P_PGID, pgid, os.WEXITED)
