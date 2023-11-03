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

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from metacontrol_kb_msgs.srv import AdaptableFunctions
from metacontrol_kb_msgs.srv import AdaptableComponents
from metacontrol_kb_msgs.srv import GetComponentConfigPerformance
from metacontrol_kb_msgs.srv import GetFDPerformance
from metacontrol_kb_msgs.srv import SelectableComponentConfigs
from metacontrol_kb_msgs.srv import SelectableFDs

from std_msgs.msg import String


class ConfigurationPlanner(Node):
    """Configuration planner."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')

        self.event_sub = self.create_subscription(
            String,
            '/metacontrol_kb/events',
            self.event_cb,
            10)

        self.component_adaptable_srv = self.create_client(
            AdaptableComponents, '/metacontrol_kb/component/adaptable')

        self.function_adaptable_srv = self.create_client(
            AdaptableFunctions, '/metacontrol_kb/function/adaptable')

        self.selectable_fds_srv = self.create_client(
            SelectableFDs, '/metacontrol_kb/function_designs/selectable')

        self.selectable_c_configs_srv = self.create_client(
            SelectableFDs,
            '/metacontrol_kb/component_configuration/selectable')

        self.get_fds_performance_srv = self.create_client(
            GetFDPerformance, '/metacontrol_kb/function_designs/performance')

        self.get_c_configs_performance_srv = self.create_client(
            GetComponentConfigPerformance,
            '/metacontrol_kb/component_configuration/performance')

        self.get_logger().info('on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.event_sub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def event_cb(self, msg):
        if msg.data == 'insert':
            # get adaptable functions
            functions = self.call_service(
                self.function_adaptable_srv, AdaptableFunctions.Request())
            # get feasible fds
            for function in functions.functions:
                request = SelectableFDs.Request()
                request.function = function
                fds = self.call_service(self.selectable_fds_srv, request)

                # get fds performance
                request = GetFDPerformance.Request()
                request.fds = fds.fds
                fds = self.call_service(self.get_fds_performance_srv, request)

                # sort fds
                sorted_fds = sorted(
                    fds, key=lambda x: x.performance, reverse=True)

            # TODO: get adaptable components
            components = self.call_service(
                self.component_adaptable_srv, AdaptableComponents.Request())

            # get feasible component configs
            for component in components:
                request = SelectableComponentConfigs.Request()
                request.component = component
                c_configs = self.call_service(
                    self.selectable_c_configs_srv, request)

                # get component configs performance
                request = GetComponentConfigPerformance.Request()
                request.c_configs = c_configs.c_configs
                c_configs = self.call_service(
                    self.get_c_configs_performance_srv, request)

                # sort fds
                sorted_c_configs = sorted(
                    c_configs, key=lambda x: x.performance, reverse=True)

            # TODO: updated kb with selected fds and component configs
            pass
        pass

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
