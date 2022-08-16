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
    answer = typedb_interface.get_leaf_functions('Navigate safely')
    print('Must solve functions: ')
    for a in answer:
        print(' '+a.get("function_name").get_value())
