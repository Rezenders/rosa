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
from metacontrol_kb_msgs.srv import TaskRequest
from metacontrol_kb_msgs.srv import TasksMatched
from metacontrol_kb.typedb_model_interface import ModelInterface

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
