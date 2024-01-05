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
import sys
from datetime import datetime

from rosa_msgs.msg import Task
from rosa_msgs.msg import Component
from rosa_msgs.msg import ComponentConfig
from rosa_msgs.msg import Function
from rosa_msgs.msg import FunctionDesign
from rosa_msgs.msg import ReconfigurationPlan

from rosa_msgs.srv import AdaptableFunctions
from rosa_msgs.srv import AdaptableComponents
from rosa_msgs.srv import ComponentQuery
from rosa_msgs.srv import GetComponentParameters
from rosa_msgs.srv import GetComponentConfigPriority
from rosa_msgs.srv import GetReconfigurationPlan
from rosa_msgs.srv import GetFDPriority
from rosa_msgs.srv import ReconfigurationPlanQuery
from rosa_msgs.srv import SelectedConfig
from rosa_msgs.srv import SelectableComponentConfigs
from rosa_msgs.srv import SelectableFDs
from rosa_msgs.srv import TaskRequest
from rosa_msgs.srv import TasksMatched

from rcl_interfaces.msg import Parameter

from rosa_kb.typedb_model_interface import ModelInterface

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_interface import ROSTypeDBInterface
from ros_typedb.ros_typedb_interface import set_query_result_value

from diagnostic_msgs.msg import DiagnosticArray


def publish_event(event_type):
    def _publish_event(func):
        def inner(*args, **kwargs):
            args[0].publish_data_event(event_type)
            return func(*args, **kwargs)
        return inner
    return _publish_event


class RosaKB(ROSTypeDBInterface):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.typedb_interface_class = ModelInterface

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        config_res = super().on_configure(state)
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            30,
            callback_group=self.query_cb_group
        )

        self.task_cb_group = MutuallyExclusiveCallbackGroup()
        self.task_request_service = self.create_service(
            TaskRequest,
            self.get_name() + '/task/request',
            self.task_request_cb,
            callback_group=self.query_cb_group
        )

        self.task_selectable_service = self.create_service(
            TasksMatched,
            self.get_name() + '/task/selectable',
            self.task_selectable_cb,
            callback_group=self.query_cb_group
        )

        self.get_adaptable_functions_service = self.create_service(
            AdaptableFunctions,
            self.get_name() + '/function/adaptable',
            self.function_adaptable_cb,
            callback_group=self.query_cb_group
        )

        self.get_adaptable_components_service = self.create_service(
            AdaptableComponents,
            self.get_name() + '/component/adaptable',
            self.component_adaptable_cb,
            callback_group=self.query_cb_group
        )

        self.get_selectable_fds_service = self.create_service(
            SelectableFDs,
            self.get_name() + '/function_designs/selectable',
            self.selectable_fd_cb,
            callback_group=self.query_cb_group
        )

        self.get_selectable_c_configs_service = self.create_service(
            SelectableComponentConfigs,
            self.get_name() + '/component_configuration/selectable',
            self.selectable_c_config_cb,
            callback_group=self.query_cb_group
        )

        self.get_fds_priority_service = self.create_service(
            GetFDPriority,
            self.get_name() + '/function_designs/priority',
            self.function_design_priority_cb,
            callback_group=self.query_cb_group
        )

        self.get_c_configs_priority_service = self.create_service(
            GetComponentConfigPriority,
            self.get_name() + '/component_configuration/priority',
            self.component_configuration_priority_cb,
            callback_group=self.query_cb_group
        )

        self.select_configuration_service = self.create_service(
            SelectedConfig,
            self.get_name() + '/select_configuration',
            self.select_configuration_cb,
            callback_group=self.query_cb_group
        )

        self.reconfig_plan_cb_group = MutuallyExclusiveCallbackGroup()
        self.get_reconfiguration_plan_service = self.create_service(
            GetReconfigurationPlan,
            self.get_name() + '/reconfiguration_plan/get',
            self.get_reconfiguration_plan_cb,
            callback_group=self.query_cb_group
        )

        self.get_latest_reconfiguration_plan_service = self.create_service(
            GetReconfigurationPlan,
            self.get_name() + '/reconfiguration_plan/get_latest',
            self.get_latest_reconfiguration_plan_cb,
            callback_group=self.query_cb_group
        )

        self.set_reconfiguration_plan_result_service = self.create_service(
            ReconfigurationPlanQuery,
            self.get_name() + '/reconfiguration_plan/result/set',
            self.set_reconfiguration_plan_result_service_cb,
            callback_group=self.query_cb_group
        )

        self.component_active_cb_group = MutuallyExclusiveCallbackGroup()
        self.set_component_active_service = self.create_service(
            ComponentQuery,
            self.get_name() + '/component/active/set',
            self.set_component_active_cb,
            callback_group=self.query_cb_group
        )

        self.get_component_active_service = self.create_service(
            ComponentQuery,
            self.get_name() + '/component/active/get',
            self.get_component_active_cb,
            callback_group=self.query_cb_group
        )

        self.get_component_parameters_service = self.create_service(
            GetComponentParameters,
            self.get_name() + '/component_parameters/get',
            self.get_component_parameters_cb,
            callback_group=self.query_cb_group
        )

        return config_res

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return super().on_cleanup(state)

    @publish_event(event_type='insert_monitoring_data')
    def update_measurement(self, diagnostic_status):
        for value in diagnostic_status.values:
            self.typedb_interface.update_measured_attribute(
                value.key, value.value)

    @publish_event(event_type='insert_monitoring_data')
    def update_component_status(self, diagnostic_status):
        recover_values = ['recovered', 'ok']
        failure_values = ['false', 'failure', 'error']
        for value in diagnostic_status.values:
            if value.value.lower() in recover_values:
                self.typedb_interface.delete_component_status(
                    value.key)
                return None
            if value.value.lower() in failure_values:
                self.typedb_interface.update_component_status(
                    value.key, 'failure')

    def diagnostics_callback(self, msg):
        measurement_messages = [
            'qa status',
            'qa measurement',
            'ea status',
            'ea measurement',
            'attribute measurement']
        component_messages = [
            'component status', 'component']
        for diagnostic_status in msg.status:
            # Update measurement
            if diagnostic_status.message.lower() in measurement_messages:
                self.update_measurement(diagnostic_status)
                continue
            if diagnostic_status.message.lower() in component_messages:
                self.update_component_status(diagnostic_status)

    @publish_event(event_type='task_change')
    def task_request_cb(self, req, res):
        if req.required is True and \
          self.typedb_interface.is_task_selectable(req.task.task_name) is True:
            self.typedb_interface.request_task(req.task.task_name)
            res.success = True
        elif req.required is False:
            self.typedb_interface.cancel_task(req.task.task_name)
            res.success = True
        else:
            res.success = False
        return res

    def task_selectable_cb(self, req, res):
        selectable_tasks = self.typedb_interface.get_selectable_tasks()
        for task_name in selectable_tasks:
            task = Task()
            task.task_name = task_name
            res.tasks.append(task)
        return res

    def function_adaptable_cb(self, req, res):
        result = self.typedb_interface.get_adaptable_functions()
        res = AdaptableFunctions.Response()
        if result is not None:
            res.success = True
            for r in result:
                _f = Function()
                _f.name = r
                res.functions.append(_f)
        else:
            res.success = False
        return res

    def component_adaptable_cb(self, req, res):
        adaptable_c = self.typedb_interface.get_adaptable_components()
        for fd in req.selected_fds:
            _fd_c = self.typedb_interface.get_components_in_function_design(
                fd)
            adaptable_c.extend(c for c in _fd_c if c not in adaptable_c)

        if len(adaptable_c) == 0:
            res.success = False
            return res

        res.success = True
        res.components = [Component(name=c) for c in adaptable_c]
        return res

    def selectable_fd_cb(self, req, res):
        fds = []
        fds = self.typedb_interface.get_selectable_fds(req.function.name)
        for fd in fds:
            _fd = FunctionDesign()
            _fd.name = fd
            res.fds.append(_fd)
        res.success = True
        return res

    def selectable_c_config_cb(self, req, res):
        c_configs = []
        c_configs = self.typedb_interface.get_selectable_c_configs(
            req.component.name)
        for c_config in c_configs:
            _c_config = ComponentConfig()
            _c_config.name = c_config
            res.c_configs.append(_c_config)
        res.success = True
        return res

    def function_design_priority_cb(self, req, res):
        for fd in req.fds:
            p = self.typedb_interface.get_function_design_priority(fd.name)
            if p is not None and len(p) > 0:
                fd.priority = p[0]
            else:
                fd.priority = sys.float_info.max
            res.fds.append(fd)
        res.success = True
        return res

    def component_configuration_priority_cb(self, req, res):
        for c_config in req.c_configs:
            p = self.typedb_interface.get_component_configuration_priority(
                c_config.name)
            if p is not None and len(p) > 0:
                c_config.priority = p[0]
            else:
                c_config.priority = sys.float_info.max
            res.c_configs.append(c_config)
        res.success = True
        return res

    @publish_event(event_type='insert_reconfiguration_plan')
    def select_configuration_cb(self, req, res):
        _selected_fds = [
            (selected_fd.function_name, selected_fd.function_design_name)
            for selected_fd in req.selected_fds]
        _selected_component_configs = [
            (selected_cc.component_name,
                selected_cc.component_configuration_name)
            for selected_cc in req.selected_component_configs]
        result = self.typedb_interface.select_configuration(
            _selected_fds, _selected_component_configs)
        if result is False:
            res.success = False
        else:
            res.success = True
        return res

    def get_component_all_attributes(self, component):
        c_dict = self.typedb_interface.get_component_all_attributes(component)
        _component = Component()
        _component.name = component
        _component.status = c_dict.pop('component-status', '')
        _component.package = c_dict.pop('package', '')
        _component.executable = c_dict.pop('executable', '')
        _component.node_type = c_dict.pop('type', '')
        _component.is_active = c_dict.pop('is_active', False)
        return _component

    def reconfig_plan_dict_to_ros_msg(self, reconfig_plan_dict):
        reconfig_plan = ReconfigurationPlan()
        if reconfig_plan_dict is not False:
            for c_activate in reconfig_plan_dict['c_activate']:
                _component = self.get_component_all_attributes(c_activate)
                reconfig_plan.components_activate.append(_component)
            for c_deactivate in reconfig_plan_dict['c_deactivate']:
                _component = self.get_component_all_attributes(c_deactivate)
                reconfig_plan.components_deactivate.append(_component)
            for c_config in reconfig_plan_dict['c_config']:
                _c_config = ComponentConfig()
                _c_config.name = c_config
                reconfig_plan.component_configurations.append(_c_config)
            reconfig_plan.start_time = reconfig_plan_dict['start-time']\
                .isoformat(timespec='milliseconds')
            reconfig_plan.result = \
                self.typedb_interface.get_reconfiguration_plan_result(
                    reconfig_plan_dict['start-time'])
        return reconfig_plan

    def get_latest_reconfiguration_plan_cb(self, req, res):
        reconfig_plan_dict = \
            self.typedb_interface.get_latest_pending_reconfiguration_plan()
        if reconfig_plan_dict is not False:
            res.reconfig_plan = self.reconfig_plan_dict_to_ros_msg(
                reconfig_plan_dict)
            res.success = True
        else:
            res.success = False
        return res

    def get_reconfiguration_plan_cb(self, req, res):
        reconfig_plan_dict = self.typedb_interface.get_reconfiguration_plan(
            datetime.fromisoformat(req.start_time))
        if reconfig_plan_dict is not False:
            res.reconfig_plan = self.reconfig_plan_dict_to_ros_msg(
                reconfig_plan_dict)
            res.success = True
        else:
            res.success = False
        return res

    def set_component_active_cb(self, req, res):
        result = self.typedb_interface.activate_component(
            req.component.name, req.component.is_active)
        if result is not None and result is not False:
            res.success = True
            res.component = req.component
            res.component.is_active = req.component.is_active
        return res

    def get_component_active_cb(self, req, res):
        result = self.typedb_interface.is_component_active(
            req.component.name)
        if result is not None:
            res.success = True
            res.component = req.component
            res.component.is_active = result
        return res

    def get_component_parameters_cb(self, req, res):
        result = self.typedb_interface.get_component_parameters(
            req.c_config.name)
        if 'Component' in result and 'ComponentParameters' in result:
            res.success = True
            res.component.name = result['Component']
            for param in result['ComponentParameters']:
                _param = Parameter()
                _param.name = param['key']
                _param.value = set_query_result_value(
                    param['value'], param['type'])
                res.parameters.append(_param)
        return res

    def set_reconfiguration_plan_result_service_cb(self, req, res):
        res_update = self.typedb_interface.update_reconfiguration_plan_result(
            req.reconfig_plan.start_time, req.reconfig_plan.result)
        if res_update is not None:
            res.success = True
            res.reconfig_plan.result = req.reconfig_plan.result
        return res

    def update_outdated_reconfiguration_plans_result_cb(self, req, res):
        pass
