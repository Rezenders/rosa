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

from rosa_msgs.msg import ComponentConfiguration
from rosa_msgs.msg import FunctionDesign

from rosa_msgs.srv import AdaptableFunctions
from rosa_msgs.srv import AdaptableComponents
from rosa_msgs.srv import GetComponentConfigurationPriority
from rosa_msgs.srv import GetFunctionDesignPriority
from rosa_msgs.srv import ReconfigurationPlanQuery
from rosa_msgs.srv import SelectableComponentConfigurations
from rosa_msgs.srv import SelectableFunctionDesigns

from std_msgs.msg import String


def check_lc_active(func):
    def inner(*args, **kwargs):
        if args[0].active is True:
            return func(*args, **kwargs)
    return inner


class ConfigurationPlanner(Node):
    """Configuration planner."""

    def __init__(self, node_name, **kwargs):
        self.active = False
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
            AdaptableComponents,
            '/rosa_kb/component/adaptable',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.function_adaptable_srv = self.create_client(
            AdaptableFunctions,
            '/rosa_kb/function/adaptable',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.selectable_fds_srv = self.create_client(
            SelectableFunctionDesigns,
            '/rosa_kb/function_designs/selectable',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.selectable_c_configs_srv = self.create_client(
            SelectableComponentConfigurations,
            '/rosa_kb/component_configuration/selectable',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_fds_priority_srv = self.create_client(
            GetFunctionDesignPriority,
            '/rosa_kb/function_designs/priority',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_c_configs_priority_srv = self.create_client(
            GetComponentConfigurationPriority,
            '/rosa_kb/component_configuration/priority',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.set_reconfig_plan_srv = self.create_client(
            ReconfigurationPlanQuery,
            '/rosa_kb/reconfiguration_plan/set',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info(self.get_name() + ': on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_activate() is called.')
        self.active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.active = False
        self.destroy_subscription(self.event_sub)
        return TransitionCallbackReturn.SUCCESS

    def plan_function_adaptation(self):
        # get adaptable functions
        selected_functions_fds = []
        functions = self.call_service(
            self.function_adaptable_srv, AdaptableFunctions.Request())
        # get feasible fds
        if functions is not None:
            for function in functions.functions:
                request = SelectableFunctionDesigns.Request()
                request.function = function
                fds = self.call_service(self.selectable_fds_srv, request)
                if fds is not None:
                    # get fds priority
                    request = GetFunctionDesignPriority.Request()
                    request.fds = fds.fds
                    fds = self.call_service(
                        self.get_fds_priority_srv, request)
                    # sort fds
                    if fds is not None:
                        sorted_fds = sorted(
                            fds.fds, key=lambda x: x.priority)
                        if len(sorted_fds) > 0:
                            selected_fd = FunctionDesign()
                            selected_fd.function.name = function.name
                            selected_fd.name = \
                                sorted_fds[0].name
                            selected_functions_fds.append(selected_fd)
        return selected_functions_fds

    def plan_component_adaptation(self, selected_functions_fds=[]):
        # get adaptable components
        selected_component_configs = []
        _selected_fds = [
            fd.name for fd in selected_functions_fds]
        components = self.call_service(
            self.component_adaptable_srv,
            AdaptableComponents.Request(selected_fds=_selected_fds))
        # get feasible component configs
        if components is not None:
            for component in components.components:
                request = SelectableComponentConfigurations.Request()
                request.component = component
                c_configs = self.call_service(
                    self.selectable_c_configs_srv, request)

                if c_configs is not None:
                    # get component configs priority
                    request = GetComponentConfigurationPriority.Request()
                    request.c_configs = c_configs.c_configs
                    c_configs = self.call_service(
                        self.get_c_configs_priority_srv, request)

                    # sort fds
                    sorted_cc = sorted(
                        c_configs.c_configs,
                        key=lambda x: x.priority,
                    )
                    if len(sorted_cc) > 0:
                        selected_cc = ComponentConfiguration()
                        selected_cc.component.name = component.name
                        selected_cc.name = \
                            sorted_cc[0].name
                        selected_component_configs.append(selected_cc)

        return selected_component_configs

    def plan_adaptation(self):
        selected_functions_fds = self.plan_function_adaptation()
        selected_component_configs = self.plan_component_adaptation(
            selected_functions_fds)

        reconfig_plan = ReconfigurationPlanQuery.Request()
        reconfig_plan.reconfig_plan.function_designs = selected_functions_fds
        reconfig_plan.reconfig_plan.component_configurations = \
            selected_component_configs
        return reconfig_plan

    @check_lc_active
    def event_cb(self, msg):
        if msg.data == 'insert_monitoring_data' or msg.data == 'action_update':
            plan = self.plan_adaptation()
            # update kb with selected fds and component configs
            if len(plan.reconfig_plan.function_designs) > 0 \
               or len(plan.reconfig_plan.component_configurations) > 0 \
               or msg.data == 'action_update':
                self.call_service(
                    self.set_reconfig_plan_srv, plan)

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
