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
import traceback

from rosa_kb.rosa_kb_typedb import RosaKB


def main():
    rclpy.init()
    traceback_logger = rclpy.logging.get_logger(
        'rosa_kb_traceback_logger')

    lc_node = RosaKB('rosa_kb')

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
