#!/usr/bin/env python

from kb_interface import TypeDBInterface

if __name__ == '__main__':
    typedb_interface = TypeDBInterface()
    typedb_interface.connect_client("localhost:1729")
    typedb_interface.create_database("pipeline_inspection", force=True)
    typedb_interface.load_schema("../typeDB/schema/schema.tql")
    # typedb_interface.load_schema("example_application_schema.tql")
    typedb_interface.load_data("../typeDB/data/example_search_pipeline.tql", force=True)
    # typedb_interface.get_ok_function_groundings()
    leaf_functions = typedb_interface.get_leaf_functions('Navigate safely')
    print('Must solve functions: ')
    for function in leaf_functions:
        function_name = function.get("function_name").get_value()
        print('  Function:'+function_name)
        function_designs = typedb_interface.get_function_designs_ordered(function_name)
        for fd in function_designs:
            fd_name = fd.get("function_design_name").get_value()
            qa_type = fd.get("qa-type").get_value()
            qa_value = fd.get("qa-value").get_value()
            print('    Fd:'+fd_name,' ',qa_type, ':',qa_value)
            components = typedb_interface.get_components_from_function_design(fd_name)
            for c in components:
                component_name = c.get("component_name").get_value()
                print("      Component:", component_name)

            component_types = typedb_interface.get_component_types_from_function_design(fd_name)
            for ct in component_types:
                component_type = ct.get("component_type").get_value()
                print("      Component Type:", component_type)
                components_from_type = typedb_interface.get_components_from_component_type(component_type)
                for c in components_from_type:
                    component_name = c.get("component_name").get_value()
                    print("        Component:", component_name)
