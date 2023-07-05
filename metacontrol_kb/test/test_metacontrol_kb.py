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

from threading import Event
from threading import Thread

import sys

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from metacontrol_kb_msgs.srv import Task
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ros_typedb_msgs.srv import Query


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
            if r.attribute_name == 'measurement' \
               and r.value.double_value == 1.72:
                correct_measurement = True

        assert correct_measurement
    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
        raise exception
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_task_request():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()

        request = Task.Request()
        request.task_name = 'task1'

        response = node.call_service(node.task_req_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'match'
        query_req.query = """
            match $ea isa Task,
                has task-name "task1",
                has is-required $task-required;
            get $task-required;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_res = False
        for r in query_res.result:
            if r.attribute_name == 'task-required' \
               and r.value.bool_value is True:
                correct_res = True

        assert response.success is True and correct_res is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_metacontrol_kb_task_cancel():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_metacontrol_kb()

        request = Task.Request()
        request.task_name = 'task_required'

        response = node.call_service(node.task_cancel_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'match'
        query_req.query = """
            match $ea isa Task,
                has task-name "task_required",
                has is-required $task-required;
            get $task-required;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_res = False
        for r in query_res.result:
            if r.attribute_name == 'task-required' \
               and r.value.bool_value is False:
                correct_res = True

        assert response.success is True and correct_res is True
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
            Task, '/metacontrol_kb/task/request')

        self.task_cancel_srv = self.create_client(
            Task, '/metacontrol_kb/task/cancel')

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
