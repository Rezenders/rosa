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
"""ROS wrapper for ROSA's typedb model."""
import sys
from datetime import datetime

import rosa_msgs
from rosa_msgs.msg import Action
from rosa_msgs.msg import Component
from rosa_msgs.msg import ComponentConfiguration
from rosa_msgs.msg import Function
from rosa_msgs.msg import FunctionDesign
from rosa_msgs.msg import ReconfigurationPlan

from rosa_msgs.srv import AdaptableFunctions
from rosa_msgs.srv import AdaptableComponents
from rosa_msgs.srv import ComponentQuery
from rosa_msgs.srv import GetComponentParameters
from rosa_msgs.srv import GetComponentConfigurationPriority
from rosa_msgs.srv import GetFunctionDesignPriority
from rosa_msgs.srv import ReconfigurationPlanQuery
from rosa_msgs.srv import SelectedConfigurations
from rosa_msgs.srv import SelectableComponentConfigurations
from rosa_msgs.srv import SelectableFunctionDesigns
from rosa_msgs.srv import ActionQuery
from rosa_msgs.srv import SelectableActions

from rcl_interfaces.msg import Parameter

import rosa_kb.typedb_model_interface
from rosa_kb.typedb_model_interface import ModelInterface

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_interface import ROSTypeDBInterface
from ros_typedb.ros_typedb_interface import set_query_result_value

import diagnostic_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray


def publish_event(event_type: str):
    """Publish event (Decorator)."""
    def _publish_event(func):
        def inner(*args, **kwargs):
            args[0].publish_data_event(event_type)
            return func(*args, **kwargs)
        return inner
    return _publish_event


class RosaKB(ROSTypeDBInterface):
    """ROS lifecycle node implementing ROSA's KB."""

    def __init__(self, node_name, **kwargs):
        """Create RosaKB node, inherits from :class:`ROSTypeDBInterface`."""
        super().__init__(node_name, **kwargs)
        self.typedb_interface_class = ModelInterface

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure RosaKB when the configure transition is called.

        :return: transition result
        """
        config_res = super().on_configure(state)
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            30,
            callback_group=self.query_cb_group
        )

        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_request_service = self.create_service(
            ActionQuery,
            self.get_name() + '/action/request',
            self.action_request_cb,
            callback_group=self.query_cb_group
        )

        self.action_selectable_service = self.create_service(
            SelectableActions,
            self.get_name() + '/action/selectable',
            self.action_selectable_cb,
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
            SelectableFunctionDesigns,
            self.get_name() + '/function_designs/selectable',
            self.selectable_fd_cb,
            callback_group=self.query_cb_group
        )

        self.get_selectable_c_configs_service = self.create_service(
            SelectableComponentConfigurations,
            self.get_name() + '/component_configuration/selectable',
            self.selectable_c_config_cb,
            callback_group=self.query_cb_group
        )

        self.get_fds_priority_service = self.create_service(
            GetFunctionDesignPriority,
            self.get_name() + '/function_designs/priority',
            self.function_design_priority_cb,
            callback_group=self.query_cb_group
        )

        self.get_c_configs_priority_service = self.create_service(
            GetComponentConfigurationPriority,
            self.get_name() + '/component_configuration/priority',
            self.component_configuration_priority_cb,
            callback_group=self.query_cb_group
        )

        self.select_configuration_service = self.create_service(
            SelectedConfigurations,
            self.get_name() + '/select_configuration',
            self.select_configuration_cb,
            callback_group=self.query_cb_group
        )

        self.reconfig_plan_cb_group = MutuallyExclusiveCallbackGroup()
        self.get_reconfiguration_plan_service = self.create_service(
            ReconfigurationPlanQuery,
            self.get_name() + '/reconfiguration_plan/get',
            self.get_reconfiguration_plan_cb,
            callback_group=self.query_cb_group
        )

        self.get_latest_reconfiguration_plan_service = self.create_service(
            ReconfigurationPlanQuery,
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
        """
        Cleanup RosaKB when the cleanup transition is called.

        :return: transition result
        """
        return super().on_cleanup(state)

    @publish_event(event_type='insert_monitoring_data')
    def update_measurement(
            self,
            diagnostic_status: diagnostic_msgs.msg.DiagnosticStatus) -> None:
        """
        Update QA/EA attribute measurement.

        Update QualityAttribute or EnvironmentalAttribute attribute
        measurement. Publish 'insert_monitoring_data' event in `rosa_kb/events`
        topic when called.

        :param diagnostic_status: measurement
        """
        for value in diagnostic_status.values:
            self.typedb_interface.add_measurement(
                value.key, value.value)

    @publish_event(event_type='insert_monitoring_data')
    def update_component_status(
            self,
            diagnostic_status: diagnostic_msgs.msg.DiagnosticStatus) -> None:
        """
        Update Component status.

        Update Component status. Publish 'insert_monitoring_data' event
        in `rosa_kb/events` topic when called. Recover values: 'recovered' or
        'ok'. Failure values: 'false', 'failure', or 'error'.

        :param diagnostic_status: component status
        """
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

    def diagnostics_callback(
            self, msg: diagnostic_msgs.msg.DiagnosticArray) -> None:
        """
        Update component status or QA/EA measurement (callback).

        Callback from topic '/dianostics'. Updates component status when
        `message` field is 'component status' or 'component'. Updates QA/EA
        measurement when `message` field is 'qa status', 'qa measurement',
        'ea status', 'ea measurement', or 'attribute measurement'.

        :param msg: msg published in `/dianostics` topic
        """
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

    @publish_event(event_type='action_update')
    def action_request_cb(
        self,
        req: rosa_msgs.srv.ActionQuery.Request,
        res: rosa_msgs.srv.ActionQuery.Response
    ) -> rosa_msgs.srv.ActionQuery.Response:
        """
        Request or cancel an Action (callback).

        Callback from service `~/action/request`. Request action when
        `is_required` field is True. Cancel action when `is_required` field
        is False. Publish 'action_update' event in `~/events` topic when
        called.

        :param req: `~/action/request` service request
        :param res: `~/action/request` service response
        :return: `~/action/request` service response
        """
        _is_action_selectable = self.typedb_interface.is_action_selectable(
            req.action.name)
        if req.action.is_required is True and _is_action_selectable is True:
            self.typedb_interface.request_action(req.action.name)
            res.success = True
        elif req.action.is_required is False:
            self.typedb_interface.cancel_action(req.action.name)
            res.success = True
        else:
            res.success = False
        return res

    def action_selectable_cb(
        self,
        req: rosa_msgs.srv.SelectableActions.Request,
        res: rosa_msgs.srv.SelectableActions.Response
    ) -> rosa_msgs.srv.SelectableActions.Response:
        """
        Get selectable actions (callback).

        Callback from service `~/action/selectable`. Get selectable actions.

        :param req: `~/action/selectable` service request
        :param res: `~/action/selectable` service response
        :return: `~/action/selectable` service response
        """
        selectable_actions = self.typedb_interface.get_selectable_actions()
        for action_name in selectable_actions:
            action = Action()
            action.name = action_name
            res.actions.append(action)
        return res

    def function_adaptable_cb(
        self,
        req: rosa_msgs.srv.AdaptableFunctions.Request,
        res: rosa_msgs.srv.AdaptableFunctions.Response
    ) -> rosa_msgs.srv.AdaptableFunctions.Response:
        """
        Get adaptable functions (callback).

        Callback from service `~/function/adaptable`. Get adaptable functions.

        :param req: `~/function/adaptable` service request
        :param res: `~/function/adaptable` service response
        :return: `~/function/adaptable` service response
        """
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

    def component_adaptable_cb(
        self,
        req: rosa_msgs.srv.AdaptableComponents.Request,
        res: rosa_msgs.srv.AdaptableComponents.Response
    ) -> rosa_msgs.srv.AdaptableComponents.Response:
        """
        Get adaptable components (callback).

        Callback from service `~/component/adaptable`. Get adaptable
        components.

        :param req: `~/component/adaptable` service request
        :param res: `~/component/adaptable` service response
        :return: `~/component/adaptable` service response
        """
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

    def selectable_fd_cb(
        self,
        req: rosa_msgs.srv.SelectableFunctionDesigns.Request,
        res: rosa_msgs.srv.SelectableFunctionDesigns.Response
    ) -> rosa_msgs.srv.SelectableFunctionDesigns.Response:
        """
        Get selectable function designs (callback).

        Callback from service `~/function_designs/selectable`. Get selectable
        function designs.

        :param req: `~/function_designs/selectable` service request
        :param res: `~/function_designs/selectable` service response
        :return: `~/function_designs/selectable` service response
        """
        fds = []
        fds = self.typedb_interface.get_selectable_fds(req.function.name)
        for fd in fds:
            _fd = FunctionDesign()
            _fd.name = fd
            res.fds.append(_fd)
        res.success = True
        return res

    def selectable_c_config_cb(
        self,
        req: rosa_msgs.srv.SelectableComponentConfigurations.Request,
        res: rosa_msgs.srv.SelectableComponentConfigurations.Response
    ) -> rosa_msgs.srv.SelectableComponentConfigurations.Response:
        """
        Get selectable component configurations (callback).

        Callback from service `~/component_configuration/selectable`. Get
        selectable component configurations.

        :param req: `~/component_configuration/selectable` service request
        :param res: `~/component_configuration/selectable` service response
        :return: `~/component_configuration/selectable` service response
        """
        c_configs = []
        c_configs = self.typedb_interface.get_selectable_c_configs(
            req.component.name)
        for c_config in c_configs:
            _c_config = ComponentConfiguration()
            _c_config.name = c_config
            res.c_configs.append(_c_config)
        res.success = True
        return res

    def function_design_priority_cb(
        self,
        req: rosa_msgs.srv.GetFunctionDesignPriority.Request,
        res: rosa_msgs.srv.GetFunctionDesignPriority.Response
    ) -> rosa_msgs.srv.GetFunctionDesignPriority.Response:
        """
        Get function designs priority (callback).

        Callback from service `~/function_designs/priority`. Get
        function designs priority.

        :param req: `~/function_designs/priority` service request
        :param res: `~/function_designs/priority` service response
        :return: `~/function_designs/priority` service response
        """
        for fd in req.fds:
            p = self.typedb_interface.get_function_design_priority(fd.name)
            if p is not None and len(p) > 0:
                fd.priority = p[0]
            else:
                fd.priority = sys.float_info.max
            res.fds.append(fd)
        res.success = True
        return res

    def component_configuration_priority_cb(
        self,
        req: rosa_msgs.srv.GetComponentConfigurationPriority.Request,
        res: rosa_msgs.srv.GetComponentConfigurationPriority.Response
    ) -> rosa_msgs.srv.GetComponentConfigurationPriority.Response:
        """
        Get component configurations priority (callback).

        Callback from service `~/component_configuration/priority`. Get
        component configurations priority.

        :param req: `~/component_configuration/priority` service request
        :param res: `~/component_configuration/priority` service response
        :return: `~/component_configuration/priority` service response
        """
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
    def select_configuration_cb(
        self,
        req: rosa_msgs.srv.SelectedConfigurations.Request,
        res: rosa_msgs.srv.SelectedConfigurations.Response
    ) -> rosa_msgs.srv.SelectedConfigurations.Response:
        """
        Select configuration (callback).

        Callback from service `~/select_configuration`. Select new
        configuration for the system. Publish `insert_reconfiguration_plan` in
        `~/events` topic.

        :param req: `~/select_configuration` service request
        :param res: `~/select_configuration` service response
        :return: `~/select_configuration` service response
        """
        _selected_fds = [
            (selected_fd.function.name, selected_fd.name)
            for selected_fd in req.selected_fds]
        _selected_component_configs = [
            (selected_cc.component.name,
                selected_cc.name)
            for selected_cc in req.selected_component_configs]
        result = self.typedb_interface.select_configuration(
            _selected_fds, _selected_component_configs)
        if result is None:
            res.success = False
        else:
            res.success = True
        return res

    def get_component_all_attributes(
            self, component: str) -> rosa_msgs.msg.Component:
        """
        Get all attributes of a component.

        :param component: component name
        :return: rosa_msgs.msg.Component msg with all attributes set
        """
        c_dict = self.typedb_interface.get_component_all_attributes(component)
        _component = Component()
        _component.name = component
        _component.status = c_dict.pop('component_status', '')
        _component.package = c_dict.pop('package', '')
        _component.executable = c_dict.pop('executable', '')
        _component.node_type = c_dict.pop('type', '')
        _component.is_active = c_dict.pop('is_active', False)
        return _component

    def reconfig_plan_dict_to_ros_msg(
        self,
        reconfig_plan_dict: rosa_kb.typedb_model_interface.ReconfigPlanDict
    ) -> rosa_msgs.msg.ReconfigurationPlan:
        """
        Convert reconfig plan to :class:`rosa_msgs.msg.ReconfigurationPlan`.

        :param reconfig_plan_dict: reconfig plan dict
        :return: reconfig plan rosa msg
        """
        reconfig_plan = ReconfigurationPlan()
        if reconfig_plan_dict is not None:
            for c_activate in reconfig_plan_dict['c_activate']:
                _component = self.get_component_all_attributes(c_activate)
                reconfig_plan.components_activate.append(_component)

            for c_deactivate in reconfig_plan_dict['c_deactivate']:
                _component = self.get_component_all_attributes(c_deactivate)
                reconfig_plan.components_deactivate.append(_component)

            for c_config in reconfig_plan_dict['c_config']:
                _c_config = ComponentConfiguration()
                _c_config.name = c_config
                reconfig_plan.component_configurations.append(_c_config)

            reconfig_plan.start_time = reconfig_plan_dict['start_time']\
                .isoformat(timespec='milliseconds')

            _result = self.typedb_interface.get_reconfiguration_plan_result(
                reconfig_plan_dict['start_time'])
            if _result is not None:
                reconfig_plan.result = _result

        return reconfig_plan

    def get_latest_reconfiguration_plan_cb(
        self,
        req: rosa_msgs.srv.ReconfigurationPlanQuery.Request,
        res: rosa_msgs.srv.ReconfigurationPlanQuery.Response
    ) -> rosa_msgs.srv.ReconfigurationPlanQuery.Response:
        """
        Get latest reconfiguration plan (callback).

        Callback from service `~/reconfiguration_plan/get_latest`. Get latest
        pending reconfiguration plan.

        :param req: `~/reconfiguration_plan/get_latest` service request
        :param res: `~/reconfiguration_plan/get_latest` service response
        :return: `~/reconfiguration_plan/get_latest` service response
        """
        reconfig_plan_dict = \
            self.typedb_interface.get_latest_pending_reconfiguration_plan()
        if reconfig_plan_dict is not None:
            res.reconfig_plan = self.reconfig_plan_dict_to_ros_msg(
                reconfig_plan_dict)
            res.success = True
        else:
            res.success = False
        return res

    def get_reconfiguration_plan_cb(
        self,
        req: rosa_msgs.srv.ReconfigurationPlanQuery.Request,
        res: rosa_msgs.srv.ReconfigurationPlanQuery.Response
    ) -> rosa_msgs.srv.ReconfigurationPlanQuery.Response:
        """
        Get reconfiguration plan with requested `start_time` (callback).

        Callback from service `~/reconfiguration_plan/get`. Get reconfiguration
        plan with requested `start_time`.

        :param req: `~/reconfiguration_plan/get` service request
        :param res: `~/reconfiguration_plan/get` service response
        :return: `~/reconfiguration_plan/get` service response
        """
        reconfig_plan_dict = self.typedb_interface.get_reconfiguration_plan(
            datetime.fromisoformat(req.reconfig_plan.start_time))
        if reconfig_plan_dict is not None:
            res.reconfig_plan = self.reconfig_plan_dict_to_ros_msg(
                reconfig_plan_dict)
            res.success = True
        else:
            res.success = False
        return res

    def set_component_active_cb(
        self,
        req: rosa_msgs.srv.ComponentQuery.Request,
        res: rosa_msgs.srv.ComponentQuery.Response
    ) -> rosa_msgs.srv.ComponentQuery.Response:
        """
        Set if a component is active (callback).

        Callback from service `~/component/active/set`. Set whether a
        component is active or not according to the `is_active` field from the
        request message.

        :param req: `~/component/active/set` service request
        :param res: `~/component/active/set` service response
        :return: `~/component/active/set` service response
        """
        result = self.typedb_interface.activate_component(
            req.component.name, req.component.is_active)
        if result is not None and result is not False:
            res.success = True
            res.component = req.component
            res.component.is_active = req.component.is_active
        return res

    def get_component_active_cb(
        self,
        req: rosa_msgs.srv.ComponentQuery.Request,
        res: rosa_msgs.srv.ComponentQuery.Response
    ) -> rosa_msgs.srv.ComponentQuery.Response:
        """
        Check whether a component is active or not (callback).

        Callback from service `~/component/active/get`. Check whether a
        component is active or not

        :param req: `~/component/active/get` service request
        :param res: `~/component/active/get` service response
        :return: `~/component/active/get` service response
        """
        result = self.typedb_interface.is_component_active(
            req.component.name)
        if result is not None:
            res.success = True
            res.component = req.component
            res.component.is_active = result
        return res

    def get_component_parameters_cb(
        self,
        req: rosa_msgs.srv.GetComponentParameters.Request,
        res: rosa_msgs.srv.GetComponentParameters.Response
    ) -> rosa_msgs.srv.GetComponentParameters.Response:
        """
        Get ComponentParameters in a component configuration (callback).

        Callback from service `~/component_parameters/get`. Get
        ComponentParameters in a component configuration relationship.

        :param req: `~/component_parameters/get` service request
        :param res: `~/component_parameters/get` service response
        :return: `~/component_parameters/get` service response
        """
        result = self.typedb_interface.get_component_parameters(
            req.c_config.name)
        if 'component' in result and 'component_parameters' in result:
            res.success = True
            res.component.name = result['component']
            for param in result['component_parameters']:
                _param = Parameter()
                _param.name = param['key']
                _param.value = set_query_result_value(
                    param['value'], param['type'])
                res.parameters.append(_param)
        return res

    def set_reconfiguration_plan_result_service_cb(
        self,
        req: rosa_msgs.srv.ReconfigurationPlanQuery.Request,
        res: rosa_msgs.srv.ReconfigurationPlanQuery.Response
    ) -> rosa_msgs.srv.ReconfigurationPlanQuery.Response:
        """
        Set reconfiguration plan result (callback).

        Callback from service `~/reconfiguration_plan/result/set`. Set
        recongiration plan result.

        :param req: `~/reconfiguration_plan/result/set` service request
        :param res: `~/reconfiguration_plan/result/set` service response
        :return: `~/reconfiguration_plan/result/set` service response
        """
        res_update = self.typedb_interface.update_reconfiguration_plan_result(
            req.reconfig_plan.start_time, req.reconfig_plan.result)
        if res_update is not None:
            if req.reconfig_plan.result is True:
                self.typedb_interface\
                    .update_outdated_reconfiguration_plans_result()
            res.success = True
            res.reconfig_plan.result = req.reconfig_plan.result
        return res
