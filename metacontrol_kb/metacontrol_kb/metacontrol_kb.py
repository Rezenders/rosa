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

from ament_index_python.packages import get_package_share_directory

from metacontrol_kb.model_interface import ModelInterface

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_interface import ROSTypeDBInterface

from diagnostic_msgs.msg import DiagnosticArray


class MetacontrolKB(ROSTypeDBInterface):
    def __init__(self, node_name, schema_path='', data_path='', **kwargs):
        super().__init__(node_name, schema_path, data_path, **kwargs)
        self.typedb_interface_class = ModelInterface

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            1,
            callback_group=ReentrantCallbackGroup()
        )
        return super().on_configure(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return super().on_cleanup(state)

    # TODO: Test if this works
    def diagnostics_callback(self, msg):
        for diagnostic_status in msg.status:
            # Update measurement
            measurement_messages = [
                'QA status',
                'QA measurement',
                'EA status',
                'EA measurement',
                'Attribute measurement']
            if diagnostic_status.message in measurement_messages:
                for value in diagnostic_status.values:
                    self.typedb_interface.update_measured_attribute(
                        value.key, value.value)


def main():
    rclpy.init()

    pkg_metacontrol_kb = get_package_share_directory('metacontrol_kb')
    schema_path = os.path.join(pkg_metacontrol_kb, 'config', 'schema.tql')
    # data_path should not be hardcoded
    data_path = os.path.join(pkg_metacontrol_kb, 'config', 'suave.tql')

    lc_node = MetacontrolKB('metacontrol_kb', schema_path, data_path)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
