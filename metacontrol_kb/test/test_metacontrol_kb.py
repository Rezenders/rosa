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
import rclpy
import traceback

from threading import Thread

import sys

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState

from metacontrol_kb_msgs.msg import Component
from metacontrol_kb_msgs.msg import Function
from metacontrol_kb_msgs.msg import SelectedComponentConfig
from metacontrol_kb_msgs.msg import SelectedFunctionDesign

from metacontrol_kb_msgs.srv import AdaptableFunctions
from metacontrol_kb_msgs.srv import AdaptableComponents
from metacontrol_kb_msgs.srv import GetComponentConfigPerformance
from metacontrol_kb_msgs.srv import GetFDPerformance
from metacontrol_kb_msgs.srv import SelectedConfig
from metacontrol_kb_msgs.srv import SelectableComponentConfigs
from metacontrol_kb_msgs.srv import SelectableFDs
from metacontrol_kb_msgs.srv import TasksMatched
from metacontrol_kb_msgs.srv import TaskRequest

from ros_typedb_msgs.srv import Query

from rclpy.node import Node


@launch_pytest.fixture
def generate_test_description():
    path_to_pkg = Path(__file__).parents[1]
    path_config = path_to_pkg / 'config'
    path_test_data = path_to_pkg / 'test' / 'test_data'

    metacontrol_kb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_to_pkg / 'metacontrol_kb' / 'metacontrol_kb_typedb.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name='metacontrol_kb',
        output='screen',
        parameters=[{
            'schema_path': str(path_config / 'schema.tql'),
            'data_path': str(path_test_data / 'test_data.tql')
        }]
    )

    return launch.LaunchDescription([
        metacontrol_kb_node,
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_lc_states():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()

        configure_res = node.change_node_state(1)
        get_inactive_state_res = node.get_node_state()

        activate_res = node.change_node_state(3)
        get_active_state_res = node.get_node_state()

        assert configure_res.success is True and \
            get_inactive_state_res.current_state.id == 2 and \
            activate_res.success is True and \
            get_active_state_res.current_state.id == 3
    finally:
        rclpy.shutdown()


@pytest.mark.skip(
    reason='Bugs when publishing')
@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_diagnostics():
    rclpy.init()
    traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ''
        key_value = KeyValue()
        key_value.key = 'ea_measurement'
        key_value.value = str(1.72)
        status_msg.values.append(key_value)
        status_msg.message = 'QA status'

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = node.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)

        node.diagnostics_pub.publish(diag_msg)

        query_req = Query.Request()
        query_req.query_type = 'match'
        query_req.query = """
            match $ea isa Attribute,
                has attribute-name "ea_measurement",
                has attribute-measurement $measurement;
            get $measurement;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_measurement = False
        for r in query_res.result:
            if r.name == 'measurement' \
               and r.value.double_value == 1.72:
                correct_measurement = True

        assert correct_measurement
    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
        raise exception
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("task_name, task_required", [
    ('task1', True),
    ('task_required', False),
])
@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_task_request(task_name, task_required):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()

        request = TaskRequest.Request()
        request.task.task_name = task_name
        request.required = task_required

        response = node.call_service(node.task_req_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'match'
        query_req.query = f"""
            match $ea isa Task,
                has task-name "{task_name}",
                has is-required $task-required;
            get $task-required;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_res = False
        for r in query_res.attributes:
            if r.name == 'task-required' \
               and r.value.bool_value is task_required:
                correct_res = True

        assert response.success is True and correct_res is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_task_selectable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()

        request = TasksMatched.Request()
        response = node.call_service(node.task_selectable_srv, request)
        res_task_names = [r.task_name for r in response.tasks]
        expected_result = [
            'task_feasible', 'task_required_solved']
        assert ('task_unfeasible' not in res_task_names) \
               and all(r in res_task_names for r in expected_result)
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_functions_adaptable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.function_adaptable_srv = node.create_client(
            AdaptableFunctions, '/metacontrol_kb/function/adaptable')

        request = AdaptableFunctions.Request()
        response = node.call_service(node.function_adaptable_srv, request)
        result = [r.name for r in response.functions]
        expected_result = ['f_unsolved', 'f_always_improve']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_components_adaptable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.component_adaptable_srv = node.create_client(
            AdaptableComponents, '/metacontrol_kb/component/adaptable')

        request = AdaptableComponents.Request()
        response = node.call_service(node.component_adaptable_srv, request)
        result = [r.name for r in response.components]
        expected_result = ['c_unsolved', 'c_always_improve']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_selectable_fds():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.selectable_fds_srv = node.create_client(
            SelectableFDs, '/metacontrol_kb/function_designs/selectable')

        request = SelectableFDs.Request()

        _f = Function()
        _f.name = 'f_fd_feasible_unfeasible'
        request.function = _f

        response = node.call_service(node.selectable_fds_srv, request)
        result = [fd.name for fd in response.fds]
        expected_result = ['f_fd_feasible']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_selectable_c_configs():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.selectable_c_configs_srv = node.create_client(
            SelectableComponentConfigs,
            '/metacontrol_kb/component_configuration/selectable')

        request = SelectableComponentConfigs.Request()

        _c = Component()
        _c.name = 'c_cc_feasible_unfeasible'
        request.component = _c

        response = node.call_service(node.selectable_c_configs_srv, request)
        result = [c_config.name for c_config in response.c_configs]
        expected_result = ['c_cc_feasible']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_get_fds_performance():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.selectable_fds_srv = node.create_client(
            SelectableFDs, '/metacontrol_kb/function_designs/selectable')
        node.fd_performance_srv = node.create_client(
            GetFDPerformance, '/metacontrol_kb/function_designs/performance')

        request_fds = SelectableFDs.Request()

        _f = Function()
        _f.name = 'f_fd_feasible_unfeasible'
        request_fds.function = _f

        response_fd = node.call_service(node.selectable_fds_srv, request_fds)

        request_p = GetFDPerformance.Request()
        request_p.fds = response_fd.fds
        response_p = node.call_service(node.fd_performance_srv, request_p)
        result = [fd.performance for fd in response_p.fds]
        expected_result = [1.0]
        assert all(r in result for r in expected_result) \
            and response_p.success is True
    finally:
        rclpy.shutdown()



@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_get_component_configuration_performance():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.selectable_c_configs_srv = node.create_client(
            SelectableComponentConfigs,
            '/metacontrol_kb/component_configuration/selectable')
        node.c_configs_performance_srv = node.create_client(
            GetComponentConfigPerformance,
            '/metacontrol_kb/component_configuration/performance')

        request_c_configs = SelectableComponentConfigs.Request()

        _cc = Component()
        _cc.name = 'c_cc_feasible_unfeasible'
        request_c_configs.component = _cc

        response_c_configs = node.call_service(
            node.selectable_c_configs_srv, request_c_configs)

        request_p = GetComponentConfigPerformance.Request()
        request_p.c_configs = response_c_configs.c_configs
        response_p = node.call_service(
            node.c_configs_performance_srv, request_p)
        result = [c_config.performance for c_config in response_p.c_configs]
        expected_result = [1.0]
        assert all(r in result for r in expected_result) \
            and response_p.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_select_configuration():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()
        node.selected_config_srv = node.create_client(
            SelectedConfig, '/metacontrol_kb/select_configuration')

        selected_config = SelectedConfig.Request()

        selected_fd = SelectedFunctionDesign()
        selected_fd.function_name = 'f_reconfigure_fd'
        selected_fd.function_design_name = 'fd_reconfig_2'

        selected_cc = SelectedComponentConfig()
        selected_cc.component_name = 'component_reconfig_3'
        selected_cc.component_configuration_name = 'cp_reconfig_2'

        selected_config.selected_fds.append(selected_fd)
        selected_config.selected_component_configs.append(selected_cc)

        response_select_config = node.call_service(
            node.selected_config_srv, selected_config)

        assert response_select_config.success is True
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            1)

        self.change_state_srv = self.create_client(
            ChangeState, '/metacontrol_kb/change_state')

        self.get_state_srv = self.create_client(
            GetState, '/metacontrol_kb/get_state')

        self.query_srv = self.create_client(
            Query, '/metacontrol_kb/query')

        self.task_req_srv = self.create_client(
            TaskRequest, '/metacontrol_kb/task/request')

        self.task_selectable_srv = self.create_client(
            TasksMatched, '/metacontrol_kb/task/selectable')

    def start_node(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node),
            args=(self,))
        self.ros_spin_thread.start()

    def change_node_state(self, transition_id):
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = transition_id
        return self.call_service(self.change_state_srv, change_state_req)

    def get_node_state(self):
        get_state_req = GetState.Request()
        return self.call_service(self.get_state_srv, get_state_req)

    def call_service(self, cli, request):
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        if self.executor.spin_until_future_complete(
                future, timeout_sec=5.0) is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None

        return future.result()

    def activate_metacontrol_kb(self):
        state_req = 1
        res = self.change_node_state(state_req)
        if res.success is True:
            state_req = 3
            res = self.change_node_state(state_req)
        if res.success is False:
            self.get_logger().error(
                'State change  req error. Requested {}'.format(state_req))
