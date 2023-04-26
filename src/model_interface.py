from typedb_interface import TypeDBInterface


class ModelInterface(TypeDBInterface):
    def __init__(self, address, database_name, schema_path, data_path=None,
                 force_database=False, force_data=False,
                 function_designs_ordering_funcs=dict(),
                 default_function_design_ordering_func='function_designs_order_desc',
                 component_ordering_funcs=dict(),
                 default_component_ordering_func='component_order_asc'):

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

        self.component_ordering_funcs = component_ordering_funcs
        self.default_component_ordering_func = default_component_ordering_func

    # Request task
    def request_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'true')

    # Cancel task
    def cancel_task(self, task_name):
        return self.update_attribute_entity(
            'Task', 'task-name', task_name, 'is-required', 'false')

    # Get unsolved tasks raw
    def get_unsolved_required_tasks_raw(self):
        query = f'''
            match
                $task isa Task, has task-name $task-name,
                    has is-required true;
                not {{$task isa Task, has is-required true,
                    has task-status $task-status;
                    $task-status = "activated";}};
                get $task-name;
        '''
        return self.match_database(query)

    # Get unsolved tasks
    def get_unsolved_required_tasks(self):
        query_result = self.get_unsolved_required_tasks_raw()
        return [result.get("task-name").get_value() for result in query_result]

    # TODO: replace for implicit-task-requirement???
    # TODO: I think it might be possible to just return a list with all
    # functions, no need for a dict
    def get_functional_hierarchy_from_task(self, task_name):
        root_functions = self.get_root_functions_from_task(task_name)
        return {f: [f] + self.get_child_functions(f) for f in root_functions}

    def get_root_functions_from_task_raw(self, task_name):
        query = f'''
            match
                $task isa Task, has task-name "{task_name}";
                (task:$task, required-function:$function) isa task-requirement;
                $function isa Function, has function-name $function-name;
                get $function-name;
        '''
        return self.match_database(query)

    def get_root_functions_from_task(self, task_name):
        query_result = self.get_root_functions_from_task_raw(task_name)
        return [r.get("function-name").get_value() for r in query_result]

    def get_child_functions_raw(self, function):
        query = f'''
            match
                $function isa Function, has function-name "{function}";
                (parent-function:$function, child-function:$fc)
                    isa implicit-functional-hierarchy;
                $fc isa Function, has function-name $fc-name;
                get $fc-name;
        '''
        return self.match_database(query)

    def get_child_functions(self, function):
        query_result = self.get_child_functions_raw(function)
        return [result.get("fc-name").get_value() for result in query_result]

    def get_required_functions_raw(self):
        query = f'''
            match
                $function isa Function, has is-function-required true,
                    has function-name $function-name;
                get $function-name;
        '''
        return self.match_database(query)

    def get_required_functions(self):
        query_result = self.get_required_functions_raw()
        return [r.get("function-name").get_value() for r in query_result]

    def get_functions_not_required_anymore_raw(self):
        query = f'''
            match
                $function isa Function, has is-function-required true,
                    has function-name $function-name;
                $task isa Task;
                {{(task:$task, required-function:$function)
                    isa implicit-task-requirement;}} or
                {{(task:$task, required-function:$function)
                    isa task-requirement;}};
                not {{$task isa Task, has is-task-required true;}};
                get $function-name;
        '''
        return self.match_database(query)

    def get_functions_not_required_anymore(self):
        query_result = self.get_functions_not_required_anymore_raw()
        return [r.get("function-name").get_value() for r in query_result]

    def get_component_types_not_required_anymore_raw(self):
        query = f'''
            match
                $component isa ComponentType,
                    has is-component-type-required true,
                    has component-type $component-type;
                (function:$function, required-component:$component)
                    isa function-design;
                not {{$function isa Function, has is-function-required true;}};
                get $component-type;
        '''
        return self.match_database(query)

    def get_component_types_not_required_anymore(self):
        query_result = self.get_component_types_not_required_anymore_raw()
        return [r.get("componet-type").get_value() for r in query_result]

    def get_components_not_required_anymore_raw(self):
        query = f'''
            match
                $component isa Component, has is-component-required true,
                    has component-name $component-name;
                {{(function:$function, required-component:$component)
                    isa function-design;
                    not {{$function isa Function,
                        has is-function-required true;}};}} or
                {{(componentType:$component-type, component:$component)
                    isa component-design;
                    not {{$component-type isa ComponentType,
                        has is-component-type-required true;}};}};
                get $component-name;
        '''
        return self.match_database(query)

    def get_components_not_required_anymore(self):
        query_result = self.get_components_not_required_anymore_raw()
        return [r.get("component-name").get_value() for r in query_result]

    # fds ordered desc single qa
    def function_designs_order_desc(self, function_name):
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function:$f, required-component:$c) isa function-design;
                $fd isa function-design, has function-design-name $fd-name;
                (function-design:$fd, qa:$eqa) isa estimated-qa;
                $eqa isa EstimatedQualityAttribute, has qa-type $qa-type,
                    has qa-value $qa-value;
                get $fd-name, $qa-type, $qa-value;
                sort $qa-value desc;
        '''
        return self.match_database(query)

    # fds ordered asc single qa
    def function_designs_order_asc(self, function_name):
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function:$f, required-component:$c) isa function-design;
                $fd isa function-design, has function-design-name $fd-name;
                (function-design:$fd, qa:$eqa) isa estimated-qa;
                $eqa isa EstimatedQualityAttribute, has qa-type $qa-type,
                    has qa-value $qa-value;
                get $fd-name, $qa-type, $qa-value;
                sort $qa-value asc;
        '''
        return self.match_database(query)

    # get fd with higher estimated qa (only considering 1 qa)
    def get_function_designs_ordered_raw(self, function_name):
        _func = None
        if function_name in self.function_designs_ordering_funcs:
            _func = getattr(
                self, self.function_designs_ordering_funcs[function_name])
        else:
            _func = getattr(self, self.default_function_design_ordering_func)

        return _func(function_name)

    def get_function_designs_ordered(self, function_name):
        query_result = self.get_function_designs_ordered_raw(function_name)
        return [r.get("fd-name").get_value() for r in query_result]

    # get components from fd (only dealing with Components, not ComponentType)
    def get_components_from_function_design_raw(self, fd_name):
        query = f'''
            match
                $fd (function:$f, required-component:$component)
                    isa function-design,  has function-design-name "{fd_name}";
                $component isa Component, has component-name $component-name;
                get $component-name;
        '''
        return self.match_database(query)

    def get_components_from_function_design(self, fd_name):
        query_result = self.get_components_from_function_design_raw(fd_name)
        return [r.get("component-name").get_value() for r in query_result]

    def get_component_types_from_function_design_raw(self, fd_name):
        query = f'''
            match
                $fd (function:$f, required-component:$c)
                    isa function-design,  has function-design-name "{fd_name}";
                $c isa ComponentType, has component-type $component-type;
                get $component-type;
        '''
        return self.match_database(query)

    def get_component_types_from_function_design(self, fd_name):
        result = self.get_component_types_from_function_design_raw(fd_name)
        return [r.get("component-type").get_value() for r in result]

    def component_order_asc(self, component_type):
        query = f'''
            match
                $ct isa ComponentType, has component-type "{component_type}";
                $cd (componentType:$ct, component:$component)
                    isa component-design,
                    has component-design-priority $priority;
                $component isa Component, has component-name $component-name;
                get $component-name, $priority;
                sort $priority asc;
        '''
        return self.match_database(query)

    def component_order_desc(self, component_type):
        query = f'''
            match
                $ct isa ComponentType, has component-type "{component_type}";
                $cd (componentType:$ct, component:$component)
                    isa component-design,
                    has component-design-priority $priority;
                $component isa Component, has component-name $component-name;
                get $component-name, $priority;
                sort $priority desc;
        '''
        return self.match_database(query)

    def get_components_from_component_type_raw(self, component_type):
        _func = None
        if component_type in self.component_ordering_funcs:
            _func = getattr(
                self, self.component_ordering_funcs[component_type])
        else:
            _func = getattr(self, self.default_component_ordering_func)
        return _func(component_type)

    def get_components_from_component_type(self, component_type):
        result = self.get_components_from_component_type_raw(component_type)
        return [r.get("component-name").get_value() for r in result]

    def delete_component(self, component_name):
        query = f'''
            match $c isa Component, has component-name "{component_name}";
            delete $c isa Component;
        '''
        return self.delete_from_database(query)

    def insert_component(self, component_name):
        query = f'''
            insert $c isa Component, has component-name "{component_name}";
        '''
        return self.insert_database(query)

    def get_attribute_from_component(self, component_name, attr):
        return self.get_attribute_from_entity(
            "Component",
            "component-name",
            component_name,
            attr)

    def delete_attribute_from_component(self, component_name, attribute_name):
        return self.delete_attribute_from_entity(
            "Component",
            "component-name",
            component_name,
            attribute_name)

    def insert_attribute_component(self, component_name, attr, attr_value):
        return self.insert_attribute_entity(
            "Component",
            "component-name",
            component_name,
            attr,
            attr_value
        )

    def update_attribute_component(self, component_name, attr, attr_value):
        self.update_attribute_entity(
            "Component",
            "component-name",
            component_name,
            attr,
            attr_value)
