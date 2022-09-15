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
            "../typeDB/data/empty.tql",  # TODO:better way to handle empty data
            force_database=True,
            force_data=True,
            function_designs_ordering_funcs=function_designs_ordering_funcs,
            component_ordering_funcs=component_ordering_funcs)

    def test_insert_component(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)

        query_match = f'''
            match $c isa Component, has component-name "{component_name}";
        '''
        result_match = self.typedb_interface.match_database(query_match)
        self.typedb_interface.delete_component(component_name)

        self.assertNotEqual(result_match, [])

    def test_delete_component_status(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
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
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)

        self.typedb_interface.insert_component_status(component_name, "error")
        status = self.typedb_interface.get_component_status(component_name)
        component_status = status[0].get("component_status").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_status, "error")

    def test_update_component_status(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.insert_component_status(component_name, "error")

        self.typedb_interface.update_component_status(
            component_name, "activated")

        status = self.typedb_interface.get_component_status(component_name)
        component_status = status[0].get("component_status").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_status, "activated")

    def test_delete_component_requirement(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        query_insert = f'''
            insert $c isa Component,
            has component-name "{component_name}",
            has is-component-required true;
        '''
        self.typedb_interface.insert_database(query_insert)

        self.typedb_interface.delete_component_requirement(component_name)
        requirement = \
            self.typedb_interface.get_component_requirement(component_name)
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(requirement, [])

    def test_insert_component_requirement(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)

        self.typedb_interface.insert_component_requirement(
            component_name, "false")
        requirement = \
            self.typedb_interface.get_component_requirement(component_name)
        component_required = \
            requirement[0].get("is_component_required").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_required, False)

    def test_update_component_requirement(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.insert_component_requirement(
            component_name, "false")

        self.typedb_interface.update_component_requirement(
            component_name, "true")

        requirement = \
            self.typedb_interface.get_component_requirement(component_name)
        component_required = \
            requirement[0].get("is_component_required").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_required, True)

    def test_update_component_pid(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.insert_component_pid(component_name, 1234)

        self.typedb_interface.update_component_pid(component_name, 4321)

        pid = self.typedb_interface.get_component_pid(component_name)
        component_pid = pid[0].get("pid").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_pid, 4321)


if __name__ == '__main__':
    unittest.main()
