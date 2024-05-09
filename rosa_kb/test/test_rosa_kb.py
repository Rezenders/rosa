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


from rosa_msgs.msg import Component
from rosa_msgs.msg import ComponentConfiguration
from rosa_msgs.msg import ComponentProcess
from rosa_msgs.msg import Function
from rosa_msgs.msg import FunctionDesign

from rosa_msgs.srv import ActionQuery
from rosa_msgs.srv import ActionQueryArray
from rosa_msgs.srv import AdaptableFunctions
from rosa_msgs.srv import AdaptableComponents
from rosa_msgs.srv import ComponentQuery
from rosa_msgs.srv import ComponentProcessQuery
from rosa_msgs.srv import ComponentProcessQueryArray
from rosa_msgs.srv import FunctionQuery
from rosa_msgs.srv import FunctionDesignQuery
from rosa_msgs.srv import FunctionalRequirementQuery
from rosa_msgs.srv import GetComponentParameters
from rosa_msgs.srv import GetComponentConfigurationPriority
from rosa_msgs.srv import ReconfigurationPlanQuery
from rosa_msgs.srv import GetFunctionDesignPriority
from rosa_msgs.srv import SelectedConfigurations
from rosa_msgs.srv import SelectableComponentConfigurations
from rosa_msgs.srv import SelectableFunctionDesigns

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.srv import Query

from rclpy.node import Node


@launch_pytest.fixture
def generate_test_description():
    path_to_pkg = Path(__file__).parents[1]
    path_config = path_to_pkg / 'config'
    path_test_data = path_to_pkg / 'test' / 'test_data'

    database_name = 'test_rosa_kb'
    rosa_kb_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            str(path_to_pkg / 'rosa_kb' / 'rosa_kb_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name='rosa_kb',
        output='screen',
        parameters=[{
            'schema_path': [
                str(path_config / 'schema.tql'),
                str(path_config / 'ros_schema.tql')],
            'data_path': [str(path_test_data / 'test_data.tql')],
            'database_name': database_name
        }]
    )

    return launch.LaunchDescription([
        rosa_kb_node,
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_lc_states():
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


# @pytest.mark.skip(
#     reason='Bugs when publishing')
@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_diagnostics():
    rclpy.init()
    traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ''
        key_value = KeyValue()
        key_value.key = 'ea_measurement'
        key_value.value = str(1.72)
        status_msg.values.append(key_value)
        status_msg.message = 'QA status'

        status_msg2 = DiagnosticStatus()
        status_msg2.level = DiagnosticStatus.OK
        status_msg2.name = ''
        key_value2 = KeyValue()
        key_value2.key = 'component1'
        key_value2.value = 'failure'
        status_msg2.values.append(key_value2)
        status_msg2.message = 'component status'

        status_msg3 = DiagnosticStatus()
        status_msg3.level = DiagnosticStatus.OK
        status_msg3.name = ''
        key_value3 = KeyValue()
        key_value3.key = 'component_failure'
        key_value3.value = 'RECOVERED'
        status_msg3.values.append(key_value3)
        status_msg3.message = 'component status'

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = node.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)
        diag_msg.status.append(status_msg2)
        diag_msg.status.append(status_msg3)

        node.diagnostics_pub.publish(diag_msg)

        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = """
            match $ea isa Attribute,
                has attribute-name "ea_measurement";
                $m (measured-attribute:$ea) isa measurement,
                    has measurement-value $measurement;
            fetch $measurement;
        """
        query_res = node.call_service(node.query_srv, query_req)

        measurement = Attribute(
            name='measurement',
            label='measurement-value',
            value=ParameterValue(type=3, double_value=1.72))

        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = """
            match $c isa Component,
                has component-name "component1",
                has component-status $c_status;
            fetch $c_status;
        """
        query_res2 = node.call_service(node.query_srv, query_req)
        c_status = Attribute(
            name='c_status',
            label='component-status',
            value=ParameterValue(type=4, string_value='failure'))

        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = """
            match $c isa Component,
                has component-name "component_failure",
                has component-status $c_status;
            fetch $c_status;
        """
        query_res3 = node.call_service(node.query_srv, query_req)
        c_status2 = Attribute(
            name='c_status',
            label='component-status',
            value=ParameterValue(type=4, string_value='feasible'))

        measurement_correct = False
        for result in query_res.results:
            if measurement in result.attributes:
                measurement_correct = True
                break
        c_status_correct = False
        for result in query_res2.results:
            if c_status in result.attributes:
                c_status_correct = True
                break
        c_status2_correct = False
        for result in query_res3.results:
            if c_status2 in result.attributes:
                c_status2_correct = True
                break

        assert measurement_correct and c_status_correct and c_status2_correct
    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
        raise exception
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("name, is_required", [
    ('action1', True),
    ('action_required', False),
])
@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_action_request(name, is_required):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        request = ActionQuery.Request()
        request.action.name = name
        request.action.is_required = is_required
        request.preference = 'ea1'

        response = node.call_service(node.action_req_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = f"""
            match $ea isa Action,
                has action-name "{name}",
                has is-required $action-required;
            fetch $action-required;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_res = False
        for result in query_res.results:
            for r in result.attributes:
                if r.name == 'action-required' \
                   and r.value.bool_value is is_required:
                    correct_res = True

        if len(query_res.results) == 0 and is_required is False:
            correct_res = True

        assert response.success is True and correct_res is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_action_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.insert_srv = node.create_client(
            ActionQuery, '/rosa_kb/action/insert')

        name = 'action_insert_test'

        request = ActionQuery.Request()
        request.action.name = name

        response = node.call_service(node.insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match $action isa Action,
                has action-name "{name}";
            get;
            count;
        """
        query_res = node.call_service(node.query_srv, query_req)
        assert response.success is True and len(query_res.results) > 0
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("action_name, result", [
    ('action1', True),
    ('action_not_in_model', False),
])
@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_action_exists(action_name, result):
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.exists_srv = node.create_client(
            ActionQuery, '/rosa_kb/action/exists')

        request = ActionQuery.Request()
        request.action.name = action_name

        response = node.call_service(node.exists_srv, request)

        assert response.success == result
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_function_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.insert_srv = node.create_client(
            FunctionQuery, '/rosa_kb/function/insert')

        name = 'function_insert_test'

        request = FunctionQuery.Request()
        request.function.name = name

        response = node.call_service(node.insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match $function isa Function,
                has function-name "{name}";
                get;
                count;
        """
        query_res = node.call_service(node.query_srv, query_req)
        assert response.success is True and len(query_res.results) > 0
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_component_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.insert_srv = node.create_client(
            ComponentQuery, '/rosa_kb/component/insert')

        name = 'component_insert_test'

        request = ComponentQuery.Request()
        request.component.name = name
        request.component.package = 'rosa_kb'
        request.component.executable = 'rosa_test'
        request.component.node_type = 'LifeCycleNode'

        response = node.call_service(node.insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match $component isa LifeCycleNode,
                has component-name "{name}",
                has package "rosa_kb",
                has executable "rosa_test";
                get;
                count;
        """
        query_res = node.call_service(node.query_srv, query_req)
        assert response.success is True and len(query_res.results) > 0
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_function_design_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        f_name = 'f_function_design_insert_test'
        node.function_insert_srv = node.create_client(
            FunctionQuery, '/rosa_kb/function/insert')

        request = FunctionQuery.Request()
        request.function.name = f_name
        response = node.call_service(node.function_insert_srv, request)

        c_names = [
            'c_function_design_insert_test',
            'c2_function_design_insert_test'
        ]
        node.component_insert_srv = node.create_client(
            ComponentQuery, '/rosa_kb/component/insert')

        request = ComponentQuery.Request()
        request.component.name = c_names[0]
        request.component.package = 'package'
        request.component.executable = 'executable'
        response = node.call_service(node.component_insert_srv, request)
        request = ComponentQuery.Request()
        request.component.name = c_names[1]
        request.component.package = 'package'
        request.component.executable = 'executable'
        response = node.call_service(node.component_insert_srv, request)

        fd_name = 'function_design_insert_test'
        node.fd_insert_srv = node.create_client(
            FunctionDesignQuery, '/rosa_kb/function_design/insert')

        request = FunctionDesignQuery.Request()
        request.function_design.name = fd_name
        request.function_design.priority = 1.5
        request.function_design.function.name = f_name
        request.function_design.required_components = [
            Component(name=c_names[0]),
            Component(name=c_names[1]),
        ]
        response = node.call_service(node.fd_insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'fetch'
        query_req.query = f"""
            match
                $fd (function: $f, required-component: $c)
                    isa function-design,
                has function-design-name "{fd_name}",
                has priority $priority;
                $f has function-name $f_name;
                $c has component-name $c_name;
            fetch $priority; $f_name; $c_name;
        """
        query_res = node.call_service(node.query_srv, query_req)
        correct_function = False
        correct_components = False
        correct_priority = False
        for result in query_res.results:
            for attr in result.attributes:
                if attr.name == 'c_name' and attr.value.string_value in c_names:
                    correct_components = True
                elif attr.name == 'c_name':
                    correct_components = False

                if attr.name == 'f_name' \
                   and attr.value.string_value == 'f_function_design_insert_test':
                    correct_function = True

                if attr.name == 'priority' and attr.value.double_value == 1.5:
                    correct_priority = True

        assert response.success is True and correct_function and \
            correct_components and correct_priority
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_functional_requirement_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        a_name = 'action_insert_test'
        node.action_insert_srv = node.create_client(
            ActionQuery, '/rosa_kb/action/insert')
        request = ActionQuery.Request()
        request.action.name = a_name
        response = node.call_service(node.action_insert_srv, request)

        f_name = 'f_functional_requirement_insert_test'
        node.function_insert_srv = node.create_client(
            FunctionQuery, '/rosa_kb/function/insert')

        request = FunctionQuery.Request()
        request.function.name = f_name
        response = node.call_service(node.function_insert_srv, request)

        node.fr_insert_srv = node.create_client(
            FunctionalRequirementQuery,
            '/rosa_kb/functional_requirement/insert')

        request = FunctionalRequirementQuery.Request()
        request.functional_requirement.action.name = a_name
        request.functional_requirement.required_functions = [
            Function(name=f_name),
        ]
        response = node.call_service(node.fr_insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match
                $a has action-name "{a_name}";
                $f has function-name "{f_name}";
                $fr (action: $a, required-function: $f)
                    isa functional-requirement;
                get;
                count;
        """
        query_res = node.call_service(node.query_srv, query_req)
        assert response.success is True and len(query_res.results) > 0
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_component_process_insert():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        c_name = 'c_component_process_insert_test'
        node.component_insert_srv = node.create_client(
            ComponentQuery, '/rosa_kb/component/insert')

        request = ComponentQuery.Request()
        request.component.name = c_name
        request.component.package = 'package'
        request.component.executable = 'executable'
        response = node.call_service(node.component_insert_srv, request)

        c_process_pid = 65755
        node.component_process_insert_srv = node.create_client(
            ComponentProcessQuery, '/rosa_kb/component_process/insert')

        request = ComponentProcessQuery.Request()
        request.component_process.component.name = c_name
        request.component_process.pid = c_process_pid
        response = node.call_service(
            node.component_process_insert_srv, request)

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match
                $c has component-name "{c_name}";
                $cp (component: $c) isa component-process;
                get;
                count;
        """
        query_res = node.call_service(node.query_srv, query_req)
        assert response.success is True and len(query_res.results) > 0
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_component_process_get_active():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        c_name = 'c_component_process_insert_test'
        node.component_insert_srv = node.create_client(
            ComponentQuery, '/rosa_kb/component/insert')

        request = ComponentQuery.Request()
        request.component.name = c_name
        request.component.package = 'package'
        request.component.executable = 'executable'
        node.call_service(node.component_insert_srv, request)

        c_process_pid = 65755
        node.component_process_insert_srv = node.create_client(
            ComponentProcessQuery, '/rosa_kb/component_process/insert')

        request = ComponentProcessQuery.Request()
        request.component_process.component.name = c_name
        request.component_process.pid = c_process_pid
        node.call_service(node.component_process_insert_srv, request)

        node.component_process_get_active_srv = node.create_client(
            ComponentProcessQueryArray,
            '/rosa_kb/component_process/get_active'
        )

        response_get = node.call_service(
            node.component_process_get_active_srv,
            ComponentProcessQueryArray.Request()
        )
        assert response_get.success is True and \
            response_get.component_process[0].pid == c_process_pid and \
            response_get.component_process[0].component.name == c_name

    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_component_process_set_end_active():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        c_name = 'c_component_process_insert_test'
        node.component_insert_srv = node.create_client(
            ComponentQuery, '/rosa_kb/component/insert')

        request = ComponentQuery.Request()
        request.component.name = c_name
        request.component.package = 'package'
        request.component.executable = 'executable'
        node.call_service(node.component_insert_srv, request)

        c_process_pid = 65755
        node.component_process_insert_srv = node.create_client(
            ComponentProcessQuery, '/rosa_kb/component_process/insert')

        request = ComponentProcessQuery.Request()
        request.component_process.component.name = c_name
        request.component_process.pid = c_process_pid
        node.call_service(node.component_process_insert_srv, request)

        node.component_process_get_active_srv = node.create_client(
            ComponentProcessQueryArray,
            '/rosa_kb/component_process/get_active'
        )

        response_get = node.call_service(
            node.component_process_get_active_srv,
            ComponentProcessQueryArray.Request()
        )

        node.component_process_set_end_srv = node.create_client(
            ComponentProcessQuery,
            '/rosa_kb/component_process/end/set'
        )

        response_set_end = node.call_service(
            node.component_process_set_end_srv,
            ComponentProcessQuery.Request(
                component_process=ComponentProcess(
                    start_time=response_get.component_process[0].start_time))
        )

        query_req = Query.Request()
        query_req.query_type = 'get_aggregate'
        query_req.query = f"""
            match
                $c has component-name "{c_name}";
                $cp (component: $c) isa component-process,
                    has end-time $time;
                get;
                count;
        """
        query_res = node.call_service(node.query_srv, query_req)

        assert response_set_end.success is True and \
            len(query_res.results) > 0

    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_action_selectable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        request = ActionQueryArray.Request()
        response = node.call_service(node.action_selectable_srv, request)
        res_names = [r.name for r in response.actions]
        expected_result = [
            'action_feasible', 'action_required_solved']
        assert ('action_unfeasible' not in res_names) \
            and all(r in res_names for r in expected_result)
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_functions_adaptable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.function_adaptable_srv = node.create_client(
            AdaptableFunctions, '/rosa_kb/function/adaptable')

        request = AdaptableFunctions.Request()
        response = node.call_service(node.function_adaptable_srv, request)
        result = [r.name for r in response.functions]
        expected_result = ['f_unsolved', 'f_always_improve']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_components_adaptable():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.component_adaptable_srv = node.create_client(
            AdaptableComponents, '/rosa_kb/component/adaptable')

        request = AdaptableComponents.Request()
        response = node.call_service(node.component_adaptable_srv, request)
        result = [r.name for r in response.components]
        expected_result = ['c_unsolved', 'c_always_improve']
        assert all(r in result for r in expected_result) \
            and response.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_selectable_fds():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.selectable_fds_srv = node.create_client(
            SelectableFunctionDesigns, '/rosa_kb/function_designs/selectable')

        request = SelectableFunctionDesigns.Request()

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
def test_rosa_kb_selectable_c_configs():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.selectable_c_configs_srv = node.create_client(
            SelectableComponentConfigurations,
            '/rosa_kb/component_configuration/selectable')

        request = SelectableComponentConfigurations.Request()

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
def test_rosa_kb_get_fds_priority():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.selectable_fds_srv = node.create_client(
            SelectableFunctionDesigns, '/rosa_kb/function_designs/selectable')
        node.fd_priority_srv = node.create_client(
            GetFunctionDesignPriority, '/rosa_kb/function_designs/priority')

        request_fds = SelectableFunctionDesigns.Request()

        _f = Function()
        _f.name = 'f_fd_feasible_unfeasible'
        request_fds.function = _f

        response_fd = node.call_service(node.selectable_fds_srv, request_fds)

        request_p = GetFunctionDesignPriority.Request()
        request_p.fds = response_fd.fds
        response_p = node.call_service(node.fd_priority_srv, request_p)
        result = [fd.priority for fd in response_p.fds]
        expected_result = [1.0]
        assert all(r in result for r in expected_result) \
            and response_p.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_get_component_configuration_priority():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.selectable_c_configs_srv = node.create_client(
            SelectableComponentConfigurations,
            '/rosa_kb/component_configuration/selectable')
        node.c_configs_priority_srv = node.create_client(
            GetComponentConfigurationPriority,
            '/rosa_kb/component_configuration/priority')

        request_c_configs = SelectableComponentConfigurations.Request()

        _cc = Component()
        _cc.name = 'c_cc_feasible_unfeasible'
        request_c_configs.component = _cc

        response_c_configs = node.call_service(
            node.selectable_c_configs_srv, request_c_configs)

        request_p = GetComponentConfigurationPriority.Request()
        request_p.c_configs = response_c_configs.c_configs
        response_p = node.call_service(
            node.c_configs_priority_srv, request_p)
        result = [c_config.priority for c_config in response_p.c_configs]
        expected_result = [1.0]
        assert all(r in result for r in expected_result) \
            and response_p.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_select_configuration():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()
        node.selected_config_srv = node.create_client(
            SelectedConfigurations, '/rosa_kb/select_configuration')

        selected_config = SelectedConfigurations.Request()

        selected_fd = FunctionDesign()
        selected_fd.function.name = 'f_reconfigure_fd'
        selected_fd.name = 'fd_reconfig_2'

        selected_cc = ComponentConfiguration()
        selected_cc.component.name = 'component_reconfig_3'
        selected_cc.name = 'cp_reconfig_2'

        selected_config.selected_fds.append(selected_fd)
        selected_config.selected_component_configs.append(selected_cc)

        response_select_config = node.call_service(
            node.selected_config_srv, selected_config)

        assert response_select_config.success is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_rosa_kb_get_reconfiguration_plan():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        node.selected_config_srv = node.create_client(
            SelectedConfigurations, '/rosa_kb/select_configuration')

        selected_config = SelectedConfigurations.Request()

        selected_fd = FunctionDesign()
        selected_fd.function.name = 'f_reconfigure_fd'
        selected_fd.name = 'fd_reconfig_2'

        selected_cc = ComponentConfiguration()
        selected_cc.component.name = 'component_reconfig_3'
        selected_cc.name = 'cp_reconfig_2'

        selected_config.selected_fds.append(selected_fd)
        selected_config.selected_component_configs.append(selected_cc)

        node.call_service(node.selected_config_srv, selected_config)

        node.get_latest_reconfig_plan_srv = node.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/get_latest')
        reconfig_plan = node.call_service(
            node.get_latest_reconfig_plan_srv,
            ReconfigurationPlanQuery.Request())

        node.get_reconfig_plan_srv = node.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/get')
        reconfig_plan_2 = node.call_service(
            node.get_reconfig_plan_srv,
            ReconfigurationPlanQuery.Request(
                reconfig_plan=reconfig_plan.reconfig_plan))

        _c = Component()
        _c.name = 'component_reconfig_2'
        _c.status = 'unsolved'
        _c.node_type = 'Component'

        _cc = ComponentConfiguration()
        _cc.name = 'cp_reconfig_2'
        assert reconfig_plan.success is True \
            and _c in reconfig_plan.reconfig_plan.components_activate \
            and _cc in reconfig_plan.reconfig_plan.component_configurations \
            and reconfig_plan.reconfig_plan.start_time != '' \
            and reconfig_plan_2.success is True \
            and reconfig_plan.reconfig_plan.start_time == \
            reconfig_plan_2.reconfig_plan.start_time
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_set_get_component_activate():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        component_1 = Component()
        component_1.name = 'c_active'
        component_1.is_active = False
        cq_1 = ComponentQuery.Request()
        cq_1.component = component_1

        component_2 = Component()
        component_2.name = 'c_inactive'
        component_2.is_active = True
        cq_2 = ComponentQuery.Request()
        cq_2.component = component_2

        srv_set = node.create_client(
            ComponentQuery, '/rosa_kb/component/active/set')
        srv_get = node.create_client(
            ComponentQuery, '/rosa_kb/component/active/get')

        res_set_1 = node.call_service(srv_set, cq_1)
        res_set_2 = node.call_service(srv_set, cq_2)
        res_get_1 = node.call_service(srv_get, cq_1)
        res_get_2 = node.call_service(srv_get, cq_2)
        assert res_set_1.success is True and res_set_2.success is True and \
            res_get_1.component.is_active is False and \
            res_get_2.component.is_active is True
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_get_component_parameters_cb():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        c_config = ComponentConfiguration()
        c_config.name = 'get_cp_cc'

        srv_get = node.create_client(
            GetComponentParameters, '/rosa_kb/component_parameters/get')

        request = GetComponentParameters.Request()
        request.c_config = c_config

        result = node.call_service(srv_get, request)
        expected_params = [
            Parameter(
                name='get_cp_1',
                value=ParameterValue(type=1, bool_value=True)),
            Parameter(
                name='get_cp_2',
                value=ParameterValue(type=6, bool_array_value=[True, False])),
            Parameter(
                name='get_cp_3',
                value=ParameterValue(type=3, double_value=3.0)),
            Parameter(
                name='get_cp_4',
                value=ParameterValue(type=8, double_array_value=[3.0, 5.0])),
            Parameter(
                name='get_cp_5',
                value=ParameterValue(type=2, integer_value=10)),
            Parameter(
                name='get_cp_6',
                value=ParameterValue(type=7, integer_array_value=[10, 14])),
            Parameter(
                name='get_cp_7',
                value=ParameterValue(type=4, string_value='teste')),
            Parameter(
                name='get_cp_8',
                value=ParameterValue(
                    type=9, string_array_value=['teste', 'teste2'])),
        ]

        assert result.success is True and result.component.name == 'get_cp_c' \
            and all(p in result.parameters for p in expected_params)
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_set_reconfiguration_plan_result_service_cb():
    rclpy.init()
    try:
        node = MakeTestNode()
        node.start_node()
        node.activate_rosa_kb()

        node.selected_config_srv = node.create_client(
            SelectedConfigurations, '/rosa_kb/select_configuration')

        selected_config = SelectedConfigurations.Request()

        selected_fd = FunctionDesign()
        selected_fd.function.name = 'f_reconfigure_fd'
        selected_fd.name = 'fd_reconfig_2'

        selected_cc = ComponentConfiguration()
        selected_cc.component.name = 'component_reconfig_3'
        selected_cc.name = 'cp_reconfig_2'

        selected_config.selected_fds.append(selected_fd)
        selected_config.selected_component_configs.append(selected_cc)

        node.call_service(node.selected_config_srv, selected_config)

        node.get_latest_reconfig_plan = node.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/get_latest')
        reconfig_plan = node.call_service(
            node.get_latest_reconfig_plan, ReconfigurationPlanQuery.Request())

        node.set_reconfig_plan_result_srv = node.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/result/set')
        rp_query = ReconfigurationPlanQuery.Request()
        rp_query.reconfig_plan.start_time = \
            reconfig_plan.reconfig_plan.start_time
        rp_query.reconfig_plan.result = 'completed'
        set_result = node.call_service(
            node.set_reconfig_plan_result_srv, rp_query)

        node.get_reconfig_plan_srv = node.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/get')

        rp_req = ReconfigurationPlanQuery.Request()
        rp_req.reconfig_plan.start_time = \
            reconfig_plan.reconfig_plan.start_time
        reconfig_plan_2 = node.call_service(
            node.get_reconfig_plan_srv, rp_req)

        assert set_result.success is True and reconfig_plan_2.success is True \
            and reconfig_plan_2.reconfig_plan.result == 'completed'
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
            ChangeState, '/rosa_kb/change_state')

        self.get_state_srv = self.create_client(
            GetState, '/rosa_kb/get_state')

        self.query_srv = self.create_client(
            Query, '/rosa_kb/query')

        self.action_req_srv = self.create_client(
            ActionQuery, '/rosa_kb/action/request')

        self.action_selectable_srv = self.create_client(
            ActionQueryArray, '/rosa_kb/action/selectable')

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
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None

        return future.result()

    def activate_rosa_kb(self):
        state_req = 1
        res = self.change_node_state(state_req)
        if res.success is True:
            state_req = 3
            res = self.change_node_state(state_req)
        if res.success is False:
            self.get_logger().error(
                'State change  req error. Requested {}'.format(state_req))
