#!/usr/bin/env python

import time
import subprocess
from kb_interface import TypeDBInterface


class Executor():
    def __init__(self, typedb_interface):
        self.typedb_interface = typedb_interface
        self.component_subprocess_dict = dict()
        self.component_subprocess_retry_n = dict()  # TODO: write this info in kb?

    # TODO: This could be triggered by an event in the database
    def request_components(self, required_components):
        for components in required_components.values():
            for component in components:
                component_executor = \
                    self.typedb_interface.get_attribute_from_component(
                        component, 'component-executor-pid')
                if len(component_executor) > 0:  # TODO: move to start_subprocess?
                    _component_executor = component_executor[0].get(
                        'attribute').get_value()
                    self.start_subprocess(component, _component_executor)

    def start_subprocess(self, component_name, subprocess_name):
        try:
            component_subprocess = subprocess.Popen(subprocess_name)
            self.component_subprocess_dict[component_name] = \
                component_subprocess

            if component_subprocess.poll() is None:
                self.typedb_interface.update_component_status(
                    component_name, 'activated')
                self.typedb_interface.update_component_pid(
                    component_name, component_subprocess.pid)
        except FileNotFoundError as err:
            print(f"File {subprocess_name} not found")

    def stop_components(self, component_names):
        for component_name in component_names:
            if component_name in self.component_subprocess_dict:
                process = self.component_subprocess_dict[component_name]
                process.terminate()
                process.wait()
                self.typedb_interface.update_component_status(
                    component_name, 'deactivated')
                self.component_subprocess_dict.pop(component_name, None)
                self.component_subprocess_retry_n.pop(component_name, None)
            else:
                print("No subprocess for component: ", component_name)

    # TODO: Add test
    def monitor_components(self):
        for component in self.component_subprocess_dict:
            max_retry = 3  # TODO: get this from knowledge base
            current_try = self.component_subprocess_retry_n.get(component, 0)
            returncode = self.component_subprocess_dict[component].poll()

            requirement_list = self.typedb_interface.get_component_requirement(component)
            is_required = False
            if len(requirement_list) > 0:  #TODO: ugly ass solution
                is_required = requirement_list[0].get("is_component_required").get_value()

            if returncode is not None and current_try < max_retry \
               and is_required: # TODO: UGLY SOLUTION. IAM CRYING
                component_executor = \
                    self.typedb_interface.get_component_executor(component)
                if len(component_executor) > 0:  # TODO: move to start_subprocess?
                    _component_executor = component_executor[0].get(
                     'component_executor').get_value()
                    self.start_subprocess(component, _component_executor)
                    self.component_subprocess_retry_n[component] = current_try + 1
            elif returncode is not None and current_try >= max_retry \
               and is_required:
                self.typedb_interface.update_component_status(component, 'error')
