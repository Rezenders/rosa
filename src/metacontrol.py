#!/usr/bin/env python

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
    print("Querying leaf functions of task ", task_name)
    functions_dict = get_leaf_functions_from_task(typedb_interface, task_name)
    print(functions_dict)

    print("Querying required components for leaf functions")
    components_dict = get_required_components(typedb_interface, functions_dict)
    print(components_dict)

    return components_dict

def executor(interface, required_components):
    print(required_components)
    for components in required_components.values():
        for component in components:
            # component_executor = interface.get_component_executor(component).get('component_executor').get_value()
            component_executor = interface.get_component_executor(component)
            if len(component_executor)>0:
                print(component_executor[0].get('component_executor').get_value())

def init_typedb_interface(address, database_name, schema_path, data_path):
    typedb_interface = TypeDBInterface()
    typedb_interface.connect_client("localhost:1729")
    typedb_interface.create_database("pipeline_inspection", force=True)
    typedb_interface.load_schema("../typeDB/schema/schema.tql")
    typedb_interface.load_data("../typeDB/data/example_search_pipeline.tql", force=True)
    return typedb_interface

if __name__ == '__main__':

    typedb_interface = init_typedb_interface(
        "localhost:1729",
        "pipeline_inspection",
        "../typeDB/schema/schema.tql",
        "../typeDB/data/example_search_pipeline.tql")

    task_name = 'Search pipeline'
    print('Task: ',task_name)
    required_components = solve_task(typedb_interface, task_name)

    executor(typedb_interface, required_components)
