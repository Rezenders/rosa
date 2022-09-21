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

        self.typedb_interface.delete_attribute_from_component(
            component_name, 'component-status')
        status = self.typedb_interface.get_attribute_from_component(
            component_name, 'component-status')
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(status, [])

    def test_insert_component_status(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)

        self.typedb_interface.insert_attribute_component(
            component_name, 'component-status', "'error'")
        status = self.typedb_interface.get_attribute_from_component(
            component_name, 'component-status')
        component_status = status[0].get("attribute").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_status, "error")

    def test_update_component_status(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.insert_attribute_component(
            component_name, 'component-status', "'error'")

        self.typedb_interface.update_attribute_component(
            component_name, 'component-status', "'activated'")

        status = self.typedb_interface.get_attribute_from_component(
            component_name, 'component-status')
        component_status = status[0].get("attribute").get_value()
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

        self.typedb_interface.delete_attribute_from_component(
            component_name, 'is-component-required')
        requirement = self.typedb_interface.get_attribute_from_component(
            component_name, 'is-component-required')
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(requirement, [])

    def test_insert_component_requirement(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)

        self.typedb_interface.insert_attribute_component(
            component_name, 'is-component-required', "false")
        requirement = self.typedb_interface.get_attribute_from_component(
            component_name, 'is-component-required')
        component_required = requirement[0].get("attribute").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_required, False)

    def test_update_component_requirement(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.update_attribute_component(
            component_name, 'is-component-required', "false")

        self.typedb_interface.update_attribute_component(
            component_name, 'is-component-required', "true")

        requirement = self.typedb_interface.get_attribute_from_component(
            component_name, 'is-component-required')
        component_required = requirement[0].get("attribute").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_required, True)

    def test_update_component_pid(self):
        component_name = "component"
        self.typedb_interface.delete_component(component_name)
        self.typedb_interface.insert_component(component_name)
        self.typedb_interface.insert_attribute_component(
            component_name, 'component-executor-pid', 1234)

        self.typedb_interface.update_attribute_component(
            component_name, 'component-executor-pid', 4321)

        pid = self.typedb_interface.get_attribute_from_component(
            component_name, 'component-executor-pid')
        component_pid = pid[0].get("attribute").get_value()
        self.typedb_interface.delete_component(component_name)

        self.assertEqual(component_pid, 4321)

    def test_get_unsolved_required_tasks(self):
        self.typedb_interface.insert_entity('Task', 'task-name', 'task1')
        self.typedb_interface.insert_attribute_entity(
            'Task',
            'task-name',
            'task1',
            'is-task-required',
            'true')

        self.typedb_interface.insert_entity('Task', 'task-name', 'task2')
        self.typedb_interface.insert_attribute_entity(
            'Task',
            'task-name',
            'task2',
            'is-task-required',
            'true')

        self.typedb_interface.insert_attribute_entity(
            'Task',
            'task-name',
            'task2',
            'task-status',
            "'activated'")

        tasks_name = self.typedb_interface.get_unsolved_required_tasks()
        # task_name = tasks[0].get("task-name").get_value()
        self.typedb_interface.delete_entity('Task', 'task-name', 'task1')
        self.assertEqual(len(tasks_name), 1)
        self.assertEqual(tasks_name[0], 'task1')

    class TestPlanner(unittest.TestCase):
        pass


if __name__ == '__main__':
    unittest.main()
