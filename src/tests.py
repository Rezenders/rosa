#!/usr/bin/env python

import unittest
from executor import Executor
from kb_interface import TypeDBInterface


class TestKbInterface(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        function_designs_ordering_funcs = {
            'Execute and control AUV motion': 'function_designs_order_desc'}
        component_ordering_funcs = {'component type': 'component_order_desc'}
        self.typedb_interface = TypeDBInterface(
            "localhost:1729",
            "test_database",
            "../typeDB/schema/schema.tql",
            "../typeDB/data/empty.tql", #TODO: better way to handle empty data
            force_database=True,
            force_data=True,
            function_designs_ordering_funcs=function_designs_ordering_funcs,
            component_ordering_funcs=component_ordering_funcs)

    def test_insert_component(self):
        component_name = "component"
        self.typedb_interface.insert_component(component_name)

        query_match = f'''
            match $c isa Component, has component-name "{component_name}";
        '''
        result_match = self.typedb_interface.match_database(query_match)
        self.typedb_interface.delete_component(component_name)

        self.assertNotEqual(result_match, [])

    def test_delete_component_status(self):
        component_name = "component"
        query_insert = f'''
            insert $c isa Component,
            has component-name "{component_name}",
            has component-status "activated";
        '''
        self.typedb_interface.insert_database(query_insert)

        self.typedb_interface.delete_component_status(component_name)
        status = self.typedb_interface.get_component_status(component_name)
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(status, [])

    def test_insert_component_status(self):
        component_name = "component"
        self.typedb_interface.insert_component(component_name)

        self.typedb_interface.insert_component_status(component_name, "error")
        status = self.typedb_interface.get_component_status(component_name)
        component_status = status[0].get("component_status").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_status, "error")


if __name__ == '__main__':
    unittest.main()
