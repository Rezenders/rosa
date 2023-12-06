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

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from rosa_msgs.msg import SelectedComponentConfig
from rosa_msgs.msg import SelectedFunctionDesign

from rosa_msgs.srv import AdaptableFunctions
from rosa_msgs.srv import AdaptableComponents
from rosa_msgs.srv import GetComponentConfigPerformance
from rosa_msgs.srv import GetFDPerformance
from rosa_msgs.srv import SelectedConfig
from rosa_msgs.srv import SelectableComponentConfigs
from rosa_msgs.srv import SelectableFDs

from std_msgs.msg import String


class ConfigurationPlanner(Node):
    """Configuration planner."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.event_sub = self.create_subscription(
            String,
            '/rosa_kb/events',
            self.event_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.component_adaptable_srv = self.create_client(
            AdaptableComponents, '/rosa_kb/component/adaptable')

        self.function_adaptable_srv = self.create_client(
            AdaptableFunctions, '/rosa_kb/function/adaptable')

        self.selectable_fds_srv = self.create_client(
            SelectableFDs, '/rosa_kb/function_designs/selectable')

        self.selectable_c_configs_srv = self.create_client(
            SelectableComponentConfigs,
            '/rosa_kb/component_configuration/selectable')

        self.get_fds_performance_srv = self.create_client(
            GetFDPerformance, '/rosa_kb/function_designs/performance')

        self.get_c_configs_performance_srv = self.create_client(
            GetComponentConfigPerformance,
            '/rosa_kb/component_configuration/performance')

        self.select_configuration_srv = self.create_client(
            SelectedConfig,
            '/rosa_kb/select_configuration')

        self.get_logger().info(self.get_name() + ': on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_activate() is called.')
        return super().on_activate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.event_sub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def plan_function_adaptation(self):
        # get adaptable functions
        selected_functions_fds = []
        functions = self.call_service(
            self.function_adaptable_srv, AdaptableFunctions.Request())
        # get feasible fds
        if functions is not None:
            for function in functions.functions:
                request = SelectableFDs.Request()
                request.function = function
                fds = self.call_service(self.selectable_fds_srv, request)
                if fds is not None:
                    # get fds performance
                    request = GetFDPerformance.Request()
                    request.fds = fds.fds
                    fds = self.call_service(
                        self.get_fds_performance_srv, request)
                    # sort fds
                    if fds is not None:
                        sorted_fds = sorted(
                            fds.fds, key=lambda x: x.performance, reverse=True)
                        if len(sorted_fds) > 0:
                            selected_fd = SelectedFunctionDesign()
                            selected_fd.function_name = function.name
                            selected_fd.function_design_name = \
                                sorted_fds[0].name
                            selected_functions_fds.append(selected_fd)
        return selected_functions_fds

    def plan_component_adaptation(self, selected_functions_fds=[]):
        # get adaptable components
        selected_component_configs = []
        _selected_fds = [
            fd.function_design_name for fd in selected_functions_fds]
        components = self.call_service(
            self.component_adaptable_srv,
            AdaptableComponents.Request(selected_fds=_selected_fds))
        # get feasible component configs
        if components is not None:
            for component in components.components:
                request = SelectableComponentConfigs.Request()
                request.component = component
                c_configs = self.call_service(
                    self.selectable_c_configs_srv, request)

                if c_configs is not None:
                    # get component configs performance
                    request = GetComponentConfigPerformance.Request()
                    request.c_configs = c_configs.c_configs
                    c_configs = self.call_service(
                        self.get_c_configs_performance_srv, request)

                    # sort fds
                    sorted_cc = sorted(
                        c_configs.c_configs,
                        key=lambda x: x.performance,
                        reverse=True
                    )
                    if len(sorted_cc) > 0:
                        selected_cc = SelectedComponentConfig()
                        selected_cc.component_name = component.name
                        selected_cc.component_configuration_name = \
                            sorted_cc[0].name
                        selected_component_configs.append(selected_cc)

        return selected_component_configs

    def plan_adaptation(self):
        selected_functions_fds = self.plan_function_adaptation()
        selected_component_configs = self.plan_component_adaptation(
            selected_functions_fds)

        selected_config = SelectedConfig.Request()
        selected_config.selected_fds = selected_functions_fds
        selected_config.selected_component_configs = \
            selected_component_configs
        return selected_config

    def event_cb(self, msg):
        if msg.data == 'insert_monitoring_data':
            selected_config = self.plan_adaptation()
            # update kb with selected fds and component configs
            if len(selected_config.selected_fds) > 0 \
               or len(selected_config.selected_component_configs) > 0:
                self.call_service(
                    self.select_configuration_srv, selected_config)

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
