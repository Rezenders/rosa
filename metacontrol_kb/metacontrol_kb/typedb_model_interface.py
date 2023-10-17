# Copyright 2023 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from ros_typedb.typedb_interface import TypeDBInterface
from datetime import datetime


class ModelInterface(TypeDBInterface):
    def __init__(self, address, database_name, schema_path, data_path=None,
                 force_database=False, force_data=False,
                 function_designs_ordering_funcs=dict(),
                 default_function_design_ordering_func='get_function_design_higher_performance',
                 component_configuration_ordering_funcs=dict(),
                 default_component_configuration_ordering_func='get_component_configuration_higher_performance'):

        super().__init__(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data
        )

        # TODO: I am not sure I like this anymore
        self.function_designs_ordering_funcs = function_designs_ordering_funcs
        self.default_function_design_ordering_func = default_function_design_ordering_func

        self.component_configuration_ordering_funcs = component_configuration_ordering_funcs
        self.default_component_configuration_ordering_func = default_component_configuration_ordering_func

        # self.component_ordering_funcs = component_ordering_funcs
        # self.default_component_ordering_func = default_component_ordering_func

    # Request task
    def request_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'true')

    # Cancel task
    def cancel_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'false')

    # Update status of a task
    def update_task_status(self, task_name, task_status):
        return self.update_attribute_entity(
            'Task',
            'task-name',
            task_name,
            'task-status',
            "'{}'".format(task_status))

    # Check if a Task is required
    def is_task_required(self, task_name):
        is_required = self.get_attribute_from_entity(
             'Task', 'task-name', task_name, 'is-required')
        if len(is_required) == 0:
            return False
        return is_required[0]

    # Check if a Task is feasible
    def is_task_feasible(self, task_name):
        status = self.get_attribute_from_entity(
             'Task', 'task-name', task_name, 'task-status')
        return all(x in status for x in ['feasible'])

    def is_task_selectable(self, task_name):
        status = self.get_attribute_from_entity(
             'Task', 'task-name', task_name, 'task-status')
        return 'unfeasible' not in status

    def get_selectable_tasks(self):
        query = """
            match
            $t isa Task, has task-name $task-name;
            not {$t isa Task, has task-status 'unfeasible';};
            get $task-name;
        """
        query_result = self.match_database(query)
        return [r.get('task-name').get('value') for r in query_result]

    # Get all entities with is-required property equal to True and
    # function-status equal to 'solved' raw
    def get_instances_of_thing_with_status(self, thing, status):
        """
        Get name of instances of a certain thing that have a certain status.

        :param thing: thing type to query.
        :type thing: str
        :param status: status.
        :type status: str
        :return: Names of instances with status.
        :rtype: list[str]
        """
        query = f'''
            match
                $e isa {thing},
                    has {thing.lower()}-name $name,
                    has {thing.lower()}-status $status;
                    $status like "{status}";
                get $name;
        '''
        query_result = self.match_database(query)
        return [r.get("name").get('value') for r in query_result]

    def get_solved_functions(self):
        return self.get_instances_of_thing_with_status(
            'Function', 'solved')

    def get_solved_components(self):
        return self.get_instances_of_thing_with_status(
            'Component', 'solved')

    def get_instances_thing_always_improve(self, thing):
        """
        Get name of instances of a certain thing that have always-improve true.

        :param thing: thing type to query.
        :type thing: str
        :return: Names of instances with always-improve true.
        :rtype: list[str]
        """
        query = f'''
            match
                $f isa {thing}, has always-improve true,
                    has {thing.lower()}-name $name;
                get $name;
        '''
        query_result = self.match_database(query)
        return [r.get("name").get('value') for r in query_result]

    def get_adaptable_functions(self):
        result = self.get_instances_of_thing_with_status(
            'Function', 'unsolved|configuration error')
        result.extend(self.get_instances_thing_always_improve('Function'))
        return result

    def get_adaptable_components(self):
        result = self.get_instances_of_thing_with_status(
            'Component', 'unsolved|configuration error')
        result.extend(self.get_instances_thing_always_improve('Component'))
        return result

    # Get all entities with is-required property equal to True and
    # function-status equal to 'unsolved' raw
    def get_unsolved_entity_raw(self, entity):
        query = f'''
            match
                $e isa {entity}, has is-required true,
                    has {entity.lower()}-name $name,
                    has {entity.lower()}-status 'unsolved';
                get $name;
        '''
        return self.match_database(query)

    # Get all Functions with is-required property equal to True and
    # function-status equal to 'unsolved'
    def get_unsolved_functions(self):
        query_result = self.get_unsolved_entity_raw('Function')
        return [r.get("name").get('value') for r in query_result]

    # Get all Components with is-required property equal to True and
    # component-status equal to 'unsolved'
    def get_unsolved_components(self):
        query_result = self.get_unsolved_entity_raw('Component')
        return [r.get("name").get('value') for r in query_result]

    def update_function_design_performance(self, fd_name, value):
        return self.update_attribute_entity(
            'function-design',
            'function-design-name',
            fd_name,
            'performance',
            value)

    def update_measured_attribute(self, attribute_name, value):
        return self.update_attribute_entity(
            'Attribute',
            'attribute-name',
            attribute_name,
            'attribute-measurement',
            value)

    def get_measured_attribute(self, attribute_name):
        measurement = self.get_attribute_from_entity(
             'Attribute',
             'attribute-name',
             attribute_name,
             'attribute-measurement')
        if len(measurement) == 0:
            return None
        return measurement[0]

    def get_function_design_higher_performance(self, function_name):
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function: $f) isa function-design,
                    has function-design-name $fd-name,
                    has performance $fd-performance;
                not {{
                    $fd has function-design-status 'unfeasible';
                }};
                get $fd-name, $fd-performance;
                sort $fd-performance desc; limit 1;
        '''
        return self.match_database(query)

    def get_best_function_design_raw(self, function_name):
        _func = None
        if function_name in self.function_designs_ordering_funcs:
            _func = getattr(
                self, self.function_designs_ordering_funcs[function_name])
        else:
            _func = getattr(self, self.default_function_design_ordering_func)

        return _func(function_name)

    def get_best_function_design(self, function_name):
        query_result = self.get_best_function_design_raw(function_name)
        if len(query_result) == 0:
            return None
        else:
            return query_result[0].get("fd-name").get('value')

    def get_component_configuration_higher_performance(self, c_name):
        query = f'''
            match
                $component isa Component, has component-name "{c_name}";
                $component-configuration (component: $component)
                    isa component-configuration,
                    has performance $conf-performance,
                    has component-configuration-name $conf-name;
                not {{
                    $component-configuration
                        has component-configuration-status 'unfeasible';
                }};
                get $conf-name, $conf-performance;
                sort $conf-performance desc; limit 1;
        '''
        return self.match_database(query)

    def get_best_component_configuration_raw(self, component_name):
        _func = None
        if component_name in self.component_configuration_ordering_funcs:
            _func = getattr(
                self,
                self.component_configuration_ordering_funcs[component_name])
        else:
            _func = getattr(
                self, self.default_component_configuration_ordering_func)

        return _func(component_name)

    def get_best_component_configuration(self, function_name):
        query_result = self.get_best_component_configuration_raw(function_name)
        if len(query_result) == 0:
            return None
        else:
            return query_result[0].get("conf-name").get('value')

    # toogle fd and component config selection
    def toogle_relationship_selection(self, entity, name, value):
        is_selected = self.get_attribute_from_entity(
            entity,
            '{}-name'.format(entity.lower()),
            name,
            'is-selected')
        if len(is_selected) > 0 and is_selected[0] is value:
            return None
        value = str(value).lower()
        return self.update_attribute_entity(
            entity,
            '{}-name'.format(entity),
            name,
            'is-selected',
            value)

    # get selected fd or component config
    def get_relationship_with_attribute(
            self, entity, entity_name, relation, r_attribute, r_value):
        query = f'''
            match
                $e isa {entity},
                    has {entity.lower()}-name "{entity_name}";
                $r ($e) isa {relation},
                    has {r_attribute} {r_value},
                    has {relation}-name $name;
                get $name;
            '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    # select fd or component configuration
    def select_relationship(self, entity, e_name, relation, r_name):
        current_selected = self.get_relationship_with_attribute(
            entity, e_name, relation, 'is-selected', 'true')
        for r in current_selected:
            if r != r_name:
                self.toogle_relationship_selection(relation, r, False)

        return self.toogle_relationship_selection(
            relation, r_name, True)

    def select_function_design(self, f_name, fd_name):
        return self.select_relationship(
            'Function', f_name, 'function-design', fd_name)

    def select_component_configuration(self, c_name, cc_name):
        return self.select_relationship(
            'Component', c_name, 'component-configuration', cc_name)

    def toogle_entity_activation(self, entity, name, value):
        is_activated = self.get_attribute_from_entity(
            entity,
            '{}-name'.format(entity.lower()),
            name,
            'is-active')
        if len(is_activated) > 0 and is_activated[0] is value:
            return None
        print('set {} to value {} '.format(name, value))
        value = str(value).lower()
        return self.update_attribute_entity(
            entity,
            '{}-name'.format(entity.lower()),
            name,
            'is-active',
            value)

    def activate_component(self, c_name, value):
        return self.toogle_entity_activation('Component', c_name, value)

    def activate_component_configuration(self, c_name, cc_name, value):
        if value is True:
            current_active = self.get_relationship_with_attribute(
                'Component',
                c_name,
                'component-configuration',
                'is-active',
                'true')
            for config in current_active:
                if config != cc_name:
                    self.toogle_entity_activation(
                        'component-configuration', config, False)
        return self.toogle_entity_activation(
            'component-configuration', cc_name, value)

    def create_reconfiguration_plan(self, c_activate, c_deactivate, c_config):
        match_query = "match "
        insert_query = "insert "
        prefix_list = []

        architectural_adaptation = []
        if len(c_activate) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('Component', 'component-name', c) for c in c_activate], 'ca')
            match_query += _match_query

            insert_query += self.create_relationship_insert_query(
                'component-activation',
                {'component': _prefix_list},
                prefix='rca'
            )
            architectural_adaptation.append('rca')

        if len(c_deactivate) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('Component', 'component-name', c) for c in c_deactivate],
                'cd')
            match_query += _match_query

            insert_query += self.create_relationship_insert_query(
                'component-deactivation',
                {'component': _prefix_list},
                prefix='rcd'
            )
            architectural_adaptation.append('rcd')

        parameter_adaptation = []
        if len(c_config) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('component-configuration', 'component-configuration-name', c)
                    for c in c_config],
                'cc'
            )
            match_query += _match_query

            insert_query += self.create_relationship_insert_query(
                'parameter-adaptation',
                {'component-configuration': _prefix_list},
                prefix='rcc'
            )
            parameter_adaptation.append('rcc')

        start_time = datetime.now()
        insert_query += self.create_relationship_insert_query(
            'reconfiguration-plan',
            {
                'architectural-adaptation': architectural_adaptation,
                'parameter-adaptation': parameter_adaptation
            },
            attribute_list=[('start-time',  start_time)],
            prefix='rp'
        )

        query = match_query + insert_query
        start_time = start_time.isoformat(timespec='milliseconds')
        return self.insert_database(query), start_time

    def get_components_in_function_design(self, fd_name):
        """
        Get components in relation with a function design.

        :param fd_name: name of the function design.
        :type thing: str
        :return: Component names in relation with fd_name.
        :rtype: list[str]
        """
        query = f'''
            match
                $fd (required-component: $component) isa function-design,
                    has function-design-name "{fd_name}";
                $component isa Component, has component-name $c-name;
                get $c-name;
            '''
        query_result = self.match_database(query)
        return [r.get('c-name').get('value') for r in query_result]
