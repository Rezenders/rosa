#!/usr/bin/env python

import time
import subprocess
from kb_interface import TypeDBInterface


class Executor():
    def __init__(self, db_interface):
        self.db_interface = db_interface
        self.component_subprocesses_dict = dict()

    # TODO: This could be triggered by an event in the database
    def request_components(self, required_components):
        for components in required_components.values():
            for component in components:
                component_executor = self.db_interface.get_component_executor(component)
                if len(component_executor)>0:
                    # print(component_executor[0].get('component_executor').get_value())
                    _component_executor= component_executor[0].get('component_executor').get_value()
                    self.start_subprocess(component, _component_executor)

    def start_subprocess(self, component_name, subprocess_name):
        try:
            self.component_subprocesses_dict[component_name] = subprocess.Popen(subprocess_name)
        except FileNotFoundError as err:
            print(f"File {subprocess_name} not found")

    def stop_components(self, component_names):
        for component_name in component_names:
            process = self.component_subprocesses_dict[component_name]
            process.terminate()
            process.wait()
