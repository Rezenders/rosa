from typedb_interface import TypeDBInterface


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

    # Add a task
    # def add_task(self, task_name):
    #     return self.insert_entity('Task', 'task-name', task_name)
    #
    # # Remove a task
    # def remove_task(self, task_name):
    #     return self.delete_entity('Task', 'task-name', task_name)

    # Request task
    def request_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'true')

    # Cancel task
    def cancel_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'false')

    # Get tasks that have the "unsolved" status raw
    # def get_required_unsolved_tasks_raw(self):
    #     query = f'''
    #         match
    #             $task isa Task, has task-name $task-name,
    #                 has is-required true, has task-status 'unsolved';
    #             get $task-name;
    #     '''
    #     return self.match_database(query)
    #
    # # Get tasks that don't have the "solved" status
    # def get_required_unsolved_tasks(self):
    #     query_result = self.get_required_unsolved_tasks_raw()
    #     return [result.get("task-name").get_value() for result in query_result]

    # Get tasks that don't have the "solved" status raw
    # def get_required_tasks_not_solved_raw(self):
    #     query = f'''
    #         match
    #             $task isa Task, has task-name $task-name,
    #                 has is-required true;
    #             not {{$task isa Task, has is-required true,
    #                 has task-status $task-status;
    #                 $task-status = "solved";}};
    #             get $task-name;
    #     '''
    #     return self.match_database(query)
    #
    # # Get tasks that don't have the "solved" status
    # def get_required_tasks_not_solved(self):
    #     query_result = self.get_required_tasks_not_solved_raw()
    #     return [result.get("task-name").get_value() for result in query_result]

    # def has_required_tasks_not_solved(self):
    #     tasks_not_solved = self.get_required_tasks_not_solved()
    #     return True if len(tasks_not_solved) > 0 else False

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

    # TODO: replace for implicit-task-requirement???
    # TODO: I think it might be possible to just return a list with all
    # functions, no need for a dict
    # def get_functional_hierarchy_from_task(self, task_name):
    #     root_functions = self.get_root_functions_from_task(task_name)
    #     return {f: [f] + self.get_child_functions(f) for f in root_functions}

    # Get all Functions in a task-requirement relationship with a Task raw
    # def get_required_functions_from_task_raw(self, task_name):
    #     query = f'''
    #         match
    #             $task isa Task, has task-name "{task_name}";
    #             (task:$task, required-function:$function) isa task-requirement;
    #             $function isa Function, has function-name $function-name;
    #             get $function-name;
    #     '''
    #     return self.match_database(query)
    #
    # # Get all Functions in a task-requirement relationship with a Task
    # def get_required_functions_from_task(self, task_name):
    #     query_result = self.get_required_functions_from_task_raw(task_name)
    #     return [r.get("function-name").get_value() for r in query_result]

    # def get_child_functions_raw(self, function):
    #     query = f'''
    #         match
    #             $function isa Function, has function-name "{function}";
    #             (parent-function:$function, child-function:$fc)
    #                 isa implicit-functional-hierarchy;
    #             $fc isa Function, has function-name $fc-name;
    #             get $fc-name;
    #     '''
    #     return self.match_database(query)
    #
    # def get_child_functions(self, function):
    #     query_result = self.get_child_functions_raw(function)
    #     return [result.get("fc-name").get_value() for result in query_result]

    # Get all Functions with is-required property equal to True raw
    def get_required_functions_raw(self):
        query = f'''
            match
                $function isa Function, has is-required true,
                    has function-name $function-name;
                get $function-name;
        '''
        return self.match_database(query)

    # Get all Functions with is-required property equal to True
    def get_required_functions(self):
        query_result = self.get_required_functions_raw()
        return [r.get("function-name").get_value() for r in query_result]

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
        return [r.get("name").get_value() for r in query_result]

    # Get all Components with is-required property equal to True and
    # component-status equal to 'unsolved'
    def get_unsolved_components(self):
        query_result = self.get_unsolved_entity_raw('Component')
        return [r.get("name").get_value() for r in query_result]

    def propagate_performance(self):
        self.propagate_components_performance()
        self.propagate_function_designs_performance()

    def get_function_designs_to_propagate_performance(self):
        query = f'''
            match
                $function-design (required-component: $component)
                    isa function-design, has function-design-name $fd-name;
                not {{
                    $function-design has performance $p;
                }};
                $component has performance $c-p;
                get $fd-name;
        '''
        return [r.get("fd-name").get_value()
                for r in self.match_database(query)]

    def get_function_design_inferred_performance_sum(self, fd_name):
        query = f'''
            match
                $function-design (required-component: $component)
                    isa function-design, has function-design-name "{fd_name}";
                not {{
                    $function-design has performance $p;
                }};
                $component has performance $c-p;
                get $c-p; sum $c-p;
        '''
        return self.match_aggregate_database(query)

    def propagate_function_designs_performance(self):
        self.propagate_entity_performance(
            self.get_function_designs_to_propagate_performance,
            self.get_function_design_inferred_performance_sum,
            self.update_function_design_performance,
        )

    def get_components_to_propagate_peformance(self):
        query = f'''
            match
                $component isa Component, has component-name $component-name;
                not {{
                    $component has performance $p;
                }};
                not {{
                    (required-component: $component) isa function-design,
                        has performance $p;
                }};
                $component-configuration (component: $component)
                    isa component-configuration, has performance $p,
                    has component-configuration-status 'feasible';
                get $component-name;
        '''
        return [r.get("component-name").get_value()
                for r in self.match_database(query)]

    def get_component_inferred_performance_max(self, c_name):
        query = f'''
            match
                $component isa Component, has component-name "{c_name}";
                not {{
                    $component has performance $p;
                }};
                $component-configuration (component: $component)
                    isa component-configuration;
                $component-configuration has performance $cc_p;
                get $cc_p; max $cc_p;
        '''
        return self.match_aggregate_database(query)

    def propagate_components_performance(self):
        self.propagate_entity_performance(
            self.get_components_to_propagate_peformance,
            self.get_component_inferred_performance_max,
            self.update_component_performance,
        )

    def propagate_entity_performance(
            self, get_entity, get_inferred_performance, update_performance):
        entities = get_entity()
        for entity in entities:
            performance = get_inferred_performance(entity)
            if performance is not None:
                update_performance(entity, performance)

    def update_component_performance(self, c_name, value):
        return self.update_attribute_entity(
            'Component',
            'component-name',
            c_name,
            'performance',
            value)

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
            return query_result[0].get("fd-name").get_value()

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
            return query_result[0].get("conf-name").get_value()

    def select_function_design(self, fd_name, value='true'):
        return self.update_attribute_entity(
            'function-design',
            'function-design-name',
            fd_name,
            'is-selected',
            value)

    def select_component_configuration(self, name, value='true'):
        return self.update_attribute_entity(
            'component-configuration',
            'component-configuration-name',
            name,
            'is-selected',
            value)

    def activate_component(self, name, value='true'):
        return self.update_attribute_entity(
            'Component',
            'component-name',
            name,
            'is-active',
            value)

    def activate_component_configuration(self, name, value='true'):
        return self.update_attribute_entity(
            'component-configuration',
            'component-configuration-name',
            name,
            'is-active',
            value)
    # def get_functions_not_required_anymore_raw(self):
    #     query = f'''
    #         match
    #             $function isa Function, has is-function-required true,
    #                 has function-name $function-name;
    #             $task isa Task;
    #             {{(task:$task, required-function:$function)
    #                 isa implicit-task-requirement;}} or
    #             {{(task:$task, required-function:$function)
    #                 isa task-requirement;}};
    #             not {{$task isa Task, has is-task-required true;}};
    #             get $function-name;
    #     '''
    #     return self.match_database(query)
    #
    # def get_functions_not_required_anymore(self):
    #     query_result = self.get_functions_not_required_anymore_raw()
    #     return [r.get("function-name").get_value() for r in query_result]
    #
    # def get_component_types_not_required_anymore_raw(self):
    #     query = f'''
    #         match
    #             $component isa ComponentType,
    #                 has is-component-type-required true,
    #                 has component-type $component-type;
    #             (function:$function, required-component:$component)
    #                 isa function-design;
    #             not {{$function isa Function, has is-function-required true;}};
    #             get $component-type;
    #     '''
    #     return self.match_database(query)
    #
    # def get_component_types_not_required_anymore(self):
    #     query_result = self.get_component_types_not_required_anymore_raw()
    #     return [r.get("componet-type").get_value() for r in query_result]
    #
    # def get_components_not_required_anymore_raw(self):
    #     query = f'''
    #         match
    #             $component isa Component, has is-component-required true,
    #                 has component-name $component-name;
    #             {{(function:$function, required-component:$component)
    #                 isa function-design;
    #                 not {{$function isa Function,
    #                     has is-function-required true;}};}} or
    #             {{(componentType:$component-type, component:$component)
    #                 isa component-design;
    #                 not {{$component-type isa ComponentType,
    #                     has is-component-type-required true;}};}};
    #             get $component-name;
    #     '''
    #     return self.match_database(query)
    #
    # def get_components_not_required_anymore(self):
    #     query_result = self.get_components_not_required_anymore_raw()
    #     return [r.get("component-name").get_value() for r in query_result]
    #


    # # get components from fd (only dealing with Components, not ComponentType)
    # def get_components_from_function_design_raw(self, fd_name):
    #     query = f'''
    #         match
    #             $fd (function:$f, required-component:$component)
    #                 isa function-design,  has function-design-name "{fd_name}";
    #             $component isa Component, has component-name $component-name;
    #             get $component-name;
    #     '''
    #     return self.match_database(query)
    #
    # def get_components_from_function_design(self, fd_name):
    #     query_result = self.get_components_from_function_design_raw(fd_name)
    #     return [r.get("component-name").get_value() for r in query_result]
    #
    # def get_component_types_from_function_design_raw(self, fd_name):
    #     query = f'''
    #         match
    #             $fd (function:$f, required-component:$c)
    #                 isa function-design,  has function-design-name "{fd_name}";
    #             $c isa ComponentType, has component-type $component-type;
    #             get $component-type;
    #     '''
    #     return self.match_database(query)
    #
    # def get_component_types_from_function_design(self, fd_name):
    #     result = self.get_component_types_from_function_design_raw(fd_name)
    #     return [r.get("component-type").get_value() for r in result]
    #
    # def component_order_asc(self, component_type):
    #     query = f'''
    #         match
    #             $ct isa ComponentType, has component-type "{component_type}";
    #             $cd (componentType:$ct, component:$component)
    #                 isa component-design,
    #                 has component-design-priority $priority;
    #             $component isa Component, has component-name $component-name;
    #             get $component-name, $priority;
    #             sort $priority asc;
    #     '''
    #     return self.match_database(query)
    #
    # def component_order_desc(self, component_type):
    #     query = f'''
    #         match
    #             $ct isa ComponentType, has component-type "{component_type}";
    #             $cd (componentType:$ct, component:$component)
    #                 isa component-design,
    #                 has component-design-priority $priority;
    #             $component isa Component, has component-name $component-name;
    #             get $component-name, $priority;
    #             sort $priority desc;
    #     '''
    #     return self.match_database(query)
    #
    # def get_components_from_component_type_raw(self, component_type):
    #     _func = None
    #     if component_type in self.component_ordering_funcs:
    #         _func = getattr(
    #             self, self.component_ordering_funcs[component_type])
    #     else:
    #         _func = getattr(self, self.default_component_ordering_func)
    #     return _func(component_type)
    #
    # def get_components_from_component_type(self, component_type):
    #     result = self.get_components_from_component_type_raw(component_type)
    #     return [r.get("component-name").get_value() for r in result]
