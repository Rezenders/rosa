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
import rclpy
import traceback

from ament_index_python.packages import get_package_share_directory

from metacontrol_kb_msgs.msg import Task
from metacontrol_kb_msgs.msg import Component
from metacontrol_kb_msgs.msg import ComponentConfig
from metacontrol_kb_msgs.msg import Function
from metacontrol_kb_msgs.msg import FunctionDesign

from metacontrol_kb_msgs.srv import AdaptableFunctions
from metacontrol_kb_msgs.srv import AdaptableComponents
from metacontrol_kb_msgs.srv import GetComponentConfigPerformance
from metacontrol_kb_msgs.srv import GetReconfigurationPlan
from metacontrol_kb_msgs.srv import GetFDPerformance
from metacontrol_kb_msgs.srv import SelectedConfig
from metacontrol_kb_msgs.srv import SelectableComponentConfigs
from metacontrol_kb_msgs.srv import SelectableFDs
from metacontrol_kb_msgs.srv import TaskRequest
from metacontrol_kb_msgs.srv import TasksMatched

from metacontrol_kb.typedb_model_interface import ModelInterface

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_interface import ROSTypeDBInterface

from diagnostic_msgs.msg import DiagnosticArray


class MetacontrolKB(ROSTypeDBInterface):
    def __init__(self, node_name, schema_path='', data_path='', **kwargs):
        super().__init__(node_name, schema_path, data_path, **kwargs)
        self.typedb_interface_class = ModelInterface

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        config_res = super().on_configure(state)
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            1,
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

        self.get_fds_performance_service = self.create_service(
            GetFDPerformance,
            self.get_name() + '/function_designs/performance',
            self.function_design_performance_cb,
            callback_group=self.query_cb_group
        )

        self.get_c_configs_performance_service = self.create_service(
            GetComponentConfigPerformance,
            self.get_name() + '/component_configuration/performance',
            self.component_configuration_performance_cb,
            callback_group=self.query_cb_group
        )

        self.select_configuration_service = self.create_service(
            SelectedConfig,
            self.get_name() + '/select_configuration',
            self.select_configuration_cb,
            callback_group=self.query_cb_group
        )

        self.get_reconfiguration_plan_service = self.create_service(
            GetReconfigurationPlan,
            self.get_name() + '/reconfiguration_plan/get',
            self.get_reconfiguration_plan_cb,
            callback_group=self.query_cb_group
        )

        return config_res

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return super().on_cleanup(state)

    def diagnostics_callback(self, msg):
        measurement_messages = [
            'QA status',
            'QA measurement',
            'EA status',
            'EA measurement',
            'Attribute measurement']
        for diagnostic_status in msg.status:
            # Update measurement
            if diagnostic_status.message in measurement_messages:
                for value in diagnostic_status.values:
                    self.typedb_interface.update_measured_attribute(
                        value.key, value.value)

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
        result = self.typedb_interface.get_adaptable_components()
        if result is not None:
            res.success = True
            for r in result:
                _c = Component()
                _c.name = r
                res.components.append(_c)
        else:
            res.success = False
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

    def function_design_performance_cb(self, req, res):
        for fd in req.fds:
            p = self.typedb_interface.get_function_design_performance(fd.name)
            if p is not None and len(p) > 0:
                fd.performance = p[0]
                res.fds.append(fd)
        res.success = True
        return res

    def component_configuration_performance_cb(self, req, res):
        for c_config in req.c_configs:
            p = self.typedb_interface.get_component_configuration_performance(
                c_config.name)
            if p is not None and len(p) > 0:
                c_config.performance = p[0]
                res.c_configs.append(c_config)
        res.success = True
        return res

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

    def get_reconfiguration_plan_cb(self, req, res):
        reconfig_plan_dict = \
            self.typedb_interface.get_latest_pending_reconfiguration_plan()
        if reconfig_plan_dict is not False:
            for c_activate in reconfig_plan_dict['c_activate']:
                _component = Component()
                _component.name = c_activate
                res.reconfig_plan.components_activate.append(_component)
            for c_deactivate in reconfig_plan_dict['c_deactivate']:
                _component = Component()
                _component.name = c_deactivate
                res.reconfig_plan.components_deactivate.append(_component)
            for c_config in reconfig_plan_dict['c_config']:
                _c_config = ComponentConfig()
                _c_config.name = c_config
                res.reconfig_plan.component_configurations.append(_c_config)
            res.success = True
        else:
            res.success = False
        return res


def main():
    rclpy.init()
    traceback_logger = rclpy.logging.get_logger(
        'metacontrol_kb_traceback_logger')

    pkg_metacontrol_kb = get_package_share_directory('metacontrol_kb')
    schema_path = os.path.join(pkg_metacontrol_kb, 'config', 'schema.tql')
    # TODO: data_path should not be hardcoded
    data_path = os.path.join(pkg_metacontrol_kb, 'config', 'suave.tql')

    lc_node = MetacontrolKB('metacontrol_kb', schema_path, data_path)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
        raise exception
    finally:
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
