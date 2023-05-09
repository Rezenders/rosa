#!/usr/bin/env python
import time
import math
from model_interface import ModelInterface

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
        'water visibility', water_visibility)
    print("Measured water visibility {}".format(water_visibility))


def analyze(kb_interface):
    # Status propagation solved by the inference reasoner
    # TODO: include adaptation to improve performance
    pass


# This should be included in the knowledge model
# the reconfig plan should include the components and component conf that
# needs to be deactivated
reconfiguration_plan = dict()


def plan(kb_interface, always_improve=True):
    # Propagate performance
    kb_interface.propagate_performance()

    adaptable_functions = list()
    adaptable_functions.extend(kb_interface.get_adaptable_functions())
    if always_improve is True:
        # TODO: remove functions without more than 1 feasible fd
        adaptable_functions.extend(kb_interface.get_solved_functions())
    for function in adaptable_functions:
        best_fd = kb_interface.get_best_function_design(function)
        if best_fd is not None:
            kb_interface.select_function_design(function, best_fd)

    adaptable_components = list()
    adaptable_components.extend(kb_interface.get_adaptable_components())
    if always_improve is True:
        # TODO: remove components without more than 1 feasible component config
        adaptable_components.extend(kb_interface.get_solved_components())
    for component in adaptable_components:
        best_config = kb_interface.get_best_component_configuration(component)
        if best_config is not None:
            kb_interface.select_component_configuration(component, best_config)
        reconfiguration_plan[component] = ('activate', best_config)


def execute(kb_interface):
    for reconf_action in reconfiguration_plan.items():
        component = reconf_action[0]
        action = reconf_action[1]
        if action[0] == 'activate':
            kb_interface.activate_component(component, True)
            if action[1] is not None:
                kb_interface.activate_component_configuration(
                    component, action[1], True)
        else:
            kb_interface.activate_component(component, False)
    reconfiguration_plan.clear()


def main():
    kb_interface = ModelInterface(
        "localhost:1729",
        "suave",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/suave.tql",
        force_database=True,
        force_data=True)

    request_result = request_task(kb_interface, 'search pipeline')
    if request_result is False:
        return False

    while True:
        monitor(kb_interface)
        analyze(kb_interface)
        plan(kb_interface)
        execute(kb_interface)
        time.sleep(1)


if __name__ == '__main__':
    main()
