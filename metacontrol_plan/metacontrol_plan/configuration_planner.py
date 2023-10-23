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

# from rcl_interfaces.msg import ParameterValue
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import TransitionCallbackReturn

# from ros_typedb.typedb_interface import TypeDBInterface
# from ros_typedb_msgs.msg import QueryResult
# from ros_typedb_msgs.srv import Query

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
            # TODO: get adaptable functions
            # TODO: get fds performance
            # TODO: select best fds

            # TODO: get adaptable components
            # TODO: get components config performance
            # TODO: select best component config

            # TODO: updated kb with selected fds and component configs
            pass
        pass
