#!/usr/bin/env python

from kb_interface import TypeDBInterface

if __name__ == '__main__':
    typedb_interface = TypeDBInterface()
    typedb_interface.connect_client("localhost:1729")
    typedb_interface.create_database("test_metacontrol")
    typedb_interface.load_schema("tomasys_schema.tql")
    typedb_interface.load_schema("example_application_schema.tql")
    typedb_interface.load_data("example_tomasys_model.tql", force=True)
    typedb_interface.get_ok_function_groundings()
