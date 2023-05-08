#!/usr/bin/env python
import time
import math
from model_interface import ModelInterface

initial_time = time.time()
water_visibility_period = 100
water_visibility_min = 1.25
water_visibility_max = 3.75
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


def analyze(kb_interface):
    # Status propagation solved by the inference reasone
    # TODO: include adaptation to improve performance
    pass


# This should be included in the knowledge model
# the reconfig plan should include the components and component conf that
# needs to be deactivated
reconfiguration_plan = dict()


def plan(kb_interface):
    # TODO: include plan to improve peformance, and overcome config errors
    kb_interface.propagate_performance()
    unsolved_functions = kb_interface.get_unsolved_functions()
    print('unsolved functions: {}'.format(unsolved_functions))
    for function in unsolved_functions:
        best_fd = kb_interface.get_best_function_design(function)
        if best_fd is not None:
            kb_interface.select_function_design(best_fd, 'true')

    unsolved_components = kb_interface.get_unsolved_components()
    print('unsolved components: {}'.format(unsolved_components))
    for component in unsolved_components:
        best_config = kb_interface.get_best_component_configuration(component)
        kb_interface.select_component_configuration(best_config, 'true')
        reconfiguration_plan[component] = ('activate', best_config)


def execute(kb_interface):
    for reconf_action in reconfiguration_plan.items():
        component = reconf_action[0]
        action = reconf_action[1]
        if action[0] == 'activate':
            kb_interface.activate_component(component, 'true')
            if action[1] is not None:
                kb_interface.activate_component_configuration(action[1], 'true')
            print('Component {0} set to {1} state, with config {2}'.format(
                component, action[0], action[1]))
        else:
            kb_interface.activate_component(component, 'false')
            print('Component {0} set to {1} state'.format(componet, action[0]))
        component_status = kb_interface.get_attribute_from_entity(
            'Component',
            'component-name',
            component,
            'component-status')
        print('Component status {}'.format(component_status))
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
        measured_wv = kb_interface.get_measured_attribute('water visibility')
        print("Measured water visibility {}".format(measured_wv))
        analyze(kb_interface)
        plan(kb_interface)
        execute(kb_interface)
        time.sleep(1)


if __name__ == '__main__':
    main()
