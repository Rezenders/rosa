#!/usr/bin/env python
import time
from planner import Planner
from executor import Executor
from kb_interface import TypeDBInterface

# def solve_task(interface, task_name):
#     functions_dict = get_leaf_functions_from_task(typedb_interface, task_name)
#     components_dict = get_required_components(typedb_interface, functions_dict)
#     return components_dict

if __name__ == '__main__':

    function_designs_ordering_funcs = {'Execute and control AUV motion':'function_designs_order_desc'}
    component_ordering_funcs = {'component type':'component_order_desc'}
    typedb_interface = TypeDBInterface(
        "localhost:1729",
        "pipeline_inspection",
        "../typeDB/schema/schema.tql",
        "../typeDB/data/example_search_pipeline.tql",
        force_database=True,
        force_data=True,
        function_designs_ordering_funcs=function_designs_ordering_funcs,
        component_ordering_funcs=component_ordering_funcs)

    planner = Planner(typedb_interface)
    executor = Executor(typedb_interface)

    # Plan
    planner.plan()
    # required_components = solve_task(typedb_interface, task_name)

    # Execute
    # executor.request_components(required_components)

    # i = 0
    # while i < 20:
    #     executor.monitor_components()
    #     time.sleep(1)
    #     i += 1
    #
    # executor.stop_components(['Thruster controller 1'])
