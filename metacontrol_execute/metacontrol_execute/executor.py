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
import subprocess
import shlex

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from std_msgs.msg import String
from metacontrol_kb_msgs.srv import GetReconfigurationPlan


class Executor(Node):
    """Executor."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.event_sub = self.create_subscription(
            String,
            '/metacontrol_kb/events',
            self.event_cb,
            10)

        self.get_reconfig_plan_srv = self.create_client(
            GetReconfigurationPlan, '/metacontrol_kb/reconfiguration_plan/get')

        self.get_logger().info(self.get_name() + ': on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_activate() is called.')
        return super().on_activate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.event_sub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def event_cb(self, msg):
        if msg.data == 'insert':
            reconfig_plan = self.call_service(
                self.get_reconfig_plan_srv, GetReconfigurationPlan.Request())
            result_deactivation = self.deactivate_components(
                reconfig_plan.components_deactivate)
            result_activation = self.activate_components(
                reconfig_plan.components_activate)
            result_update = self.update_component_params(
                reconfig_plan.component_configurations)
            result = result_deactivation and result_activation \
                and result_update
            # TODO: update reconfig plan result

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

    def deactivate_components(self, components):
        pass

    def activate_components(self, components):
        pass

    def update_component_params(self, configurations):
        pass

    def start_ros_node(self, node_dict):
        cmd = 'ros2 run '
        if 'package' in node_dict and 'executable' in node_dict:
            cmd += node_dict['package'] + ' '
            cmd += node_dict['executable']
            if 'name' in node_dict or 'parameters' in node_dict:
                cmd += ' --ros-args '
                if 'name' in node_dict:
                    cmd += ' -r __node:=' + node_dict['name']
                if 'parameters' in node_dict:
                    for param in node_dict['parameters']:
                        for key, value in param.items():
                            cmd += ' -r ' + str(key) + ':=' + str(value)
            return self.run_process(cmd, 'start_ros_node', node_dict)
        else:
            return False

    def start_ros_launchfile(self, launch_dict, **kwargs):
        cmd = 'ros2 launch '
        if 'package' in launch_dict and 'launch_file' in launch_dict:
            cmd += launch_dict['package'] + ' '
            cmd += launch_dict['launch_file']
            if 'parameters' in launch_dict:
                if 'parameters' in launch_dict:
                    for param in launch_dict['parameters']:
                        for key, value in param.items():
                            cmd += ' ' + str(key) + ':=' + str(value)
            return self.run_process(cmd, 'start_ros_launchfile', launch_dict)
        else:
            return False

    def run_process(self, cmd, _func, _dict):
        try:
            process = subprocess.Popen(
                shlex.split(cmd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            try:
                outs, errs = process.communicate(timeout=.5)
                self.get_logger().error(f'''
                    {_func} failed!
                    input _dict: {_dict}
                    resulting erros: {errs}''')
                return False
            except subprocess.TimeoutExpired:
                return process
        except Exception as e:
            self.get_logger().error(f'''
                {_func} failed!
                input _dict: {_dict}
                raised exception: {e}''')
            return False
