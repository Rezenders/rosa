#!/usr/bin/env python
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from metacontrol_kb.typedb_model_interface import ModelInterface

initial_time = time.time()
water_visibility_period = 80
water_visibility_min = 1.75
water_visibility_max = 3.5
water_visibility_amp = abs(water_visibility_max - water_visibility_min)/2
sec_shift = 0.0


def request_task(kb_interface, task_name):
    if kb_interface.is_task_feasible(task_name) is True:
        kb_interface.request_task(task_name)
        return True
    else:
        return False


def calculate_water_visibility():
    current_time = time.time()
    t = current_time - initial_time
    v_delta = water_visibility_amp + water_visibility_min
    water_visibility = water_visibility_amp * math.cos(
        (2*math.pi/water_visibility_period)*(t + sec_shift)) + v_delta
    return water_visibility


def monitor(kb_interface):
    water_visibility = calculate_water_visibility()
    kb_interface.update_measured_attribute(
        'water_visibility', water_visibility)
    print("Measured water visibility {}".format(water_visibility))


def analyze(kb_interface):
    # Status propagation solved by the inference reasoner
    # TODO: include adaptation to improve performance
    pass


def plan(kb_interface, always_improve=True):
    adaptable_functions = list()
    adaptable_functions.extend(kb_interface.get_adaptable_functions())
    print('adaptable_functions ', adaptable_functions)
    selected_functions_fds = []
    for function in adaptable_functions:
        best_fd = kb_interface.get_best_function_design(function)
        if best_fd is not None:
            selected_functions_fds.append((function, best_fd))

    adaptable_components = list()
    adaptable_components.extend(kb_interface.get_adaptable_components())
    print('adaptable_components ', adaptable_components)
    c_config_list = []
    selected_component_configs = []
    for component in adaptable_components:
        best_config = kb_interface.get_best_component_configuration(component)
        if best_config is not None:
            selected_component_configs.append((component, best_config))
    print('selected_functions_fds: ', selected_functions_fds)
    print('selected_component_configs: ', selected_component_configs)
    kb_interface.select_configuration(
        selected_functions_fds, selected_component_configs)


def execute(kb_interface):
    # TODO: get latest plan without result
    reconfiguration_plan = kb_interface.get_latest_pending_reconfiguration_plan()
    print('reconfiguration plan: ', reconfiguration_plan)
    if reconfiguration_plan is False:
        return False
    reconfig_plan_result = True
    if 'c_activate' in reconfiguration_plan:
        for component in reconfiguration_plan['c_activate']:
            r = kb_interface.activate_component(component, True)
            if r is None or r is False:
                reconfig_plan_result = False

    if 'c_deactivate' in reconfiguration_plan:
        for component in reconfiguration_plan['c_deactivate']:
            r = kb_interface.activate_component(component, False)
            if r is None or r is False:
                reconfig_plan_result = False

    if 'c_config' in reconfiguration_plan:
        for config in reconfiguration_plan['c_deactivate']:
            pass  # get parameters and set them

    kb_interface.update_reconfiguration_plan_result(
        reconfiguration_plan['start-time'], 'completed')
    kb_interface.update_outdated_reconfiguration_plans_result()


def main():
    pkg_mc_kb_path = get_package_share_directory('metacontrol_kb')
    schema_path = os.path.join(
        pkg_mc_kb_path,
        'config',
        'schema.tql')
    suave_data_path = os.path.join(
        pkg_mc_kb_path,
        'config',
        'suave.tql')
    kb_interface = ModelInterface(
        'localhost:1729',
        'suave',
        schema_path,
        suave_data_path,
        force_database=True,
        force_data=True)

    request_result = request_task(kb_interface, 'search_pipeline')
    print('request_result: ', request_result)
    if request_result is False:
        return False

    while True:
        print('Monitor')
        monitor(kb_interface)
        print('Analyze')
        analyze(kb_interface)
        print('Plan')
        plan(kb_interface)
        print('Execute')
        execute(kb_interface)
        time.sleep(1)


if __name__ == '__main__':
    main()
