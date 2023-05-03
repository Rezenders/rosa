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


def analyze():
    # Status propagation solved by the inference reasone
    # TODO: include adaptation to improve performance
    pass


def plan(kb_interface):
    # TODO: implement plan when task is unsolved
    # if task status = unsolved (not needed)
    #   get unsolved functions
    #       select fd
    #   get unsolved fds
    #       request components
    #   get unsolved components
    #       select component configuration

    unsolved_functions = kb_interface.get_unsolved_required_functions()
    pass


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

        time.sleep(1)


if __name__ == '__main__':
    main()
