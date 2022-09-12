#!/usr/bin/env python
import time
from executor import Executor
from kb_interface import TypeDBInterface

# get leaf functions from task
def get_leaf_functions_from_task(interface, task_name):
    functions = interface.get_leaf_functions_from_task(task_name)
    functions_dict = dict()
    for f in functions:
        root_function = f.get("function_name").get_value()
        leaf_function = f.get("leaf_function_name").get_value()
        if root_function in functions_dict:
            functions_dict[root_function] += [leaf_function]
        else:
            functions_dict[root_function] = [leaf_function]
    return functions_dict

def get_required_components(interface, functions_dict):
    components_dict = dict()
    for key in functions_dict:
        required_functions = functions_dict[key]
        for function in required_functions:
            function_designs = interface.get_function_designs_ordered(function)

            #TODO: Don't throw exception when len=0. would be nice to get an explanition of why
            function_design_name = function_designs[0].get("function_design_name").get_value() #TODO: check if feasible? order?
            required_components =[]
            required_components_type = interface.get_component_types_from_function_design(function_design_name)

            for component_type in required_components_type:
                _component_type_str = component_type.get("component_type").get_value()
                _components_designs_ordered = interface.get_components_from_component_type(_component_type_str)
                required_components += [{'component_name':_components_designs_ordered[0].get("component_name")}]

            required_components += interface.get_components_from_function_design(function_design_name)
            for component in required_components:
                if function in components_dict:
                    components_dict[function] += [component.get("component_name").get_value()]
                else:
                    components_dict[function] = [component.get("component_name").get_value()]
    return components_dict

def solve_task(interface, task_name):
    functions_dict = get_leaf_functions_from_task(typedb_interface, task_name)
    components_dict = get_required_components(typedb_interface, functions_dict)
    return components_dict

if __name__ == '__main__':

    function_designs_ordering_funcs = {'Execute and control AUV motion':'function_designs_order_desc'}
    typedb_interface = TypeDBInterface(
        "localhost:1729",
        "pipeline_inspection",
        "../typeDB/schema/schema.tql",
        "../typeDB/data/example_search_pipeline.tql",
        force_database=True,
        force_data=True,
        function_designs_ordering_funcs=function_designs_ordering_funcs)

    executor = Executor(typedb_interface)
    task_name = 'Search pipeline'
    print('Task: ',task_name)

    # Plan
    required_components = solve_task(typedb_interface, task_name)

    # Execute
    executor.request_components(required_components)

    time.sleep(2)
    executor.stop_components(['Thruster controller 1'])
