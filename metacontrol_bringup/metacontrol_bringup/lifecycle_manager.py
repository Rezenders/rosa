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

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState


class LifeCycleManager(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.declare_parameter(
            'managed_nodes',
            ['metacontrol_kb', 'configuration_planner', 'executor'])
        self.managed_nodes = self.get_parameter('managed_nodes').value
        self.change_state_dict = dict()
        self.get_state_dict = dict()
        self.cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        for node in self.managed_nodes:
            self.change_state_dict[node] = self.create_client(
                ChangeState,
                node + '/change_state',
                callback_group=self.cb_group)
            self.get_state_dict[node] = self.create_client(
                GetState,
                node + '/get_state',
                callback_group=self.cb_group)

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

    def change_lc_node_state(self, node_name, transition_id):
        srv = self.change_state_dict[node_name]
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = transition_id
        return self.call_service(srv, change_state_req)

    def get_lc_node_state(self, node_name):
        get_state_srv = self.get_state_dict[node_name]
        get_state_req = GetState.Request()
        return self.call_service(get_state_srv, get_state_req)

    def activate_lc_node(self, node_name):
        _state = self.get_lc_node_state(node_name)
        if _state.current_state.id == 1:
            self.change_lc_node_state(node_name, 1)
        _state = self.get_lc_node_state(node_name)
        if _state.current_state.id == 2:
            self.change_lc_node_state(node_name, 3)
        _state = self.get_lc_node_state(node_name)
        if _state.current_state.id != 3:
            return False
        return True

    def activate_managed_nodes(self):

        for node in self.managed_nodes:
            if self.activate_lc_node(node) is False:
                return False
        return False


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = LifeCycleManager('lc_manager')
    executor.add_node(lc_node)
    future = executor.create_task(lc_node.activate_managed_nodes)
    try:
        executor.spin_until_future_complete(future, timeout_sec=30.0)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
