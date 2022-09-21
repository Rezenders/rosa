from typedb.client import TypeDB
from typedb.client import TypeDBOptions
from typedb.client import SessionType
from typedb.client import TransactionType
from typedb.client import TypeDBClientException


# TODO: split into 2 classes, one with basic functionalities for interacting
# with typeDB and another one with the specific queries for "Metacontrol"
class TypeDBInterface:
    def __del__(self):
        self.client.close()

    def __init__(self, address, database_name, schema_path, data_path=None,
                 force_database=False, force_data=False,
                 function_designs_ordering_funcs=dict(),
                 default_function_design_ordering_func='function_designs_order_desc',
                 component_ordering_funcs=dict(),
                 default_component_ordering_func='component_order_asc'):
        self.connect_client(address)
        self.create_database(database_name, force=force_database)
        self.load_schema(schema_path)
        if data_path is not None:
            self.load_data(data_path, force=force_data)

        self.function_designs_ordering_funcs = function_designs_ordering_funcs
        self.default_function_design_ordering_func = default_function_design_ordering_func

        self.component_ordering_funcs = component_ordering_funcs
        self.default_component_ordering_func = default_component_ordering_func

    def connect_client(self, address, parallelisation=2):
        self.client = TypeDB.core_client(
            address=address, parallelisation=parallelisation)

    def create_database(self, database_name, force=False):
        if self.client.databases().contains(database_name) and force:
            self.client.databases().get(database_name).delete()

        if not self.client.databases().contains(database_name):
            self.database = self.client.databases().create(database_name)
            self.database_name = database_name
        else:
            self.database_name = database_name
            print("The database with the name ", database_name,
                  " already exists. Ignoring create_database request.")

    def create_session(self, database_name, session_type, options=TypeDBOptions.core()):
        return self.client.session(database_name, session_type, options)

    # Read/write database
    # Generic query method
    def database_query(self, session_type, transaction_type, query_type, query, options=TypeDBOptions.core()):
        with self.create_session(self.database_name, session_type) as session:
            options.infer = True
            with session.transaction(transaction_type, options) as transaction:
                transaction_query_function = getattr(
                    transaction.query(), query_type)
                query_answer = transaction_query_function(query)
                if transaction_type == TransactionType.WRITE:
                    transaction.commit()
                    return query_answer
                elif transaction_type == TransactionType.READ:
                    answer_list = []
                    for answer in query_answer:
                        answer_list.append(answer.map())
                    return answer_list

    # Load schema or data from file
    def write_database_file(self, session_type, transaction_type, query_type, file_path):
        with open(file_path, mode='r') as file:
            query = file.read()

        self.database_query(session_type, transaction_type, query_type, query)

    # Load schema from file
    def load_schema(self, schema_path):
        self.write_database_file(
            SessionType.SCHEMA, TransactionType.WRITE, 'define', schema_path)

    # Load data from file
    def load_data(self, data_path, force=False):
        if force:
            self.delete_from_database(
                'match $e isa entity; delete $e isa entity;')
            self.delete_from_database(
                'match $r isa relation; delete $r isa relation;')
            self.delete_from_database(
                'match $a isa attribute; delete $a isa attribute;')
        try:
            self.write_database_file(
                SessionType.DATA, TransactionType.WRITE, 'insert', data_path)
        except TypeDBClientException as err:
            print("Error in load_data method. This is the exception msg: ", err)

    # Events begining
    def insert_data_event(func):
        def inner(*args, **kwargs):
            print('Data has been inserted!')  # TODO: Do something else
            return func(*args, **kwargs)
        return inner

    def delete_data_event(func):
        def inner(*args, **kwargs):
            print('Data has been deleted!')  # TODO: Do something else
            return func(*args, **kwargs)
        return inner
    # Events end

    # Insert query
    @insert_data_event
    def insert_database(self, query):
        return self.database_query(SessionType.DATA, TransactionType.WRITE, 'insert', query)

    # Delete query
    @delete_data_event
    def delete_from_database(self, query):
        return self.database_query(SessionType.DATA, TransactionType.WRITE, 'delete', query)

    # TODO: decorator?
    # Match query
    def match_database(self, query):
        return self.database_query(SessionType.DATA, TransactionType.READ, 'match', query)

    # Read/write database end

    # Queries begining
    def get_unsolved_required_tasks_raw(self):
        query = f'''
            match
                $task isa Task, has task-name $task-name,
                    has is-task-required true;
                not {{$task isa Task, has is-task-required true,
                    has task-status $task-status;
                    $task-status = "activated";}};
                get $task-name;
        '''
        return self.match_database(query)

    def get_unsolved_required_tasks(self):
        query_result = self.get_unsolved_required_tasks_raw()
        return [result.get("task-name").get_value() for result in query_result]

    # def get_leaf_functions_from_task_raw(self, task_name):
    #     query = f'''
    #         match
    #             $task isa Task, has task-name "{task_name}";
    #             (task:$task, required-function:$function) isa task-requirement;
    #             $function isa Function, has function-name $function_name;
    #             $leaf_function_name isa function-name;
    #             {{$function has is-leaf-function true,
    #                 has function-name $leaf_function_name;}} or
    #             {{(parent-function:$function, child-function:$fc)
    #                 isa implicit-functional-hierarchy;
    #                 $fc has is-leaf-function true,
    #                 has function-name $leaf_function_name; }};
    #             get $function_name, $leaf_function_name;
    #     '''
    #     return self.match_database(query)
    #
    # def get_leaf_functions_from_task(self, task_name):
    #     query_result = self.get_leaf_functions_from_task_raw(task_name)
    #     functions_dict = dict()
    #
    #     for function in query_result:
    #         root_function = function.get("function_name").get_value()
    #         leaf_function = function.get("leaf_function_name").get_value()
    #         functions_dict.setdefault(root_function, []).append(leaf_function)
    #
    #     return functions_dict

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

    # def get_functional_hierarchy_from_task(self, task_name):
    #     query_result = self.get_functional_hierarchy_from_task_raw(task_name)
    #     functions_dict = dict()
    #
    #     for result in query_result:
    #         root_function = result.get("function-name").get_value()
    #         print(root_function)
    #         leaf_function = result.get("fc-name").get_value()
    #         print(leaf_function)
    #         functions_dict.setdefault(root_function, []).append(leaf_function)
    #
    #     return functions_dict

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

    # def get_required_functions_from_task(self, task_name):
    #     query = f'''
    #         match
    #             $t isa Task, has task-name "{task_name}";
    #             (task:$t, required-function:$function) isa task-requirement;
    #             $function isa Function, has function-name $function_name;
    #             get $function_name;
    #     '''
    #     return self.match_database(query)

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


    # TODO: standard for variable names in queries
    # def get_leaf_functions(self, root_function_name):
    #     query = f'''
    #         match
    #             $f isa Function, has function-name "{root_function_name}";
    #             (parent-function:$f, child-function:$leaf_function) isa implicit-functional-hierarchy;
    #             $leaf_function has function-name $function_name;
    #             $leaf_function has is-leaf-function true;
    #             get $leaf_function, $function_name;
    #     '''
    #     return self.match_database(query)

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

    def delete_entity(self, entity, key, key_value):
        query = f'''
            match $entity isa {entity}, has {key} "{key_value}";
            delete $entity isa {entity};
        '''
        return self.delete_from_database(query)

    def delete_component(self, component_name):
        query = f'''
            match $c isa Component, has component-name "{component_name}";
            delete $c isa Component;
        '''
        return self.delete_from_database(query)

    def insert_entity(self, entity, key, key_value):
        query = f'''
            insert $entity isa {entity}, has {key} "{key_value}";
        '''
        return self.insert_database(query)

    def insert_component(self, component_name):
        query = f'''
            insert $c isa Component, has component-name "{component_name}";
        '''
        return self.insert_database(query)

    def get_attribute_from_entity(self, entity, key, key_value, attr):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            get $attribute;
        '''
        return self.match_database(query)

    def get_attribute_from_component(self, component_name, attr):
        return self.get_attribute_from_entity(
            "Component",
            "component-name",
            component_name,
            attr)

    def delete_attribute_from_entity(self, entity, key, key_value, attr):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            delete $entity has $attribute;
        '''
        return self.delete_from_database(query)

    def delete_attribute_from_component(self, component_name, attribute_name):
        return self.delete_attribute_from_entity(
            "Component",
            "component-name",
            component_name,
            attribute_name)

    def insert_attribute_entity(
     self, entity, key, key_value, attr, attr_value):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}";
            insert $entity has {attr} {attr_value};
        '''
        return self.insert_database(query)

    def insert_attribute_component(self, component_name, attr, attr_value):
        return self.insert_attribute_entity(
            "Component",
            "component-name",
            component_name,
            attr,
            attr_value
        )

    def update_attribute_entity(
     self, entity, key, key_value, attr, attr_value):
        self.delete_attribute_from_entity(
            entity, key, key_value, attr)
        self.insert_attribute_entity(
            entity, key, key_value, attr, attr_value)

    def update_attribute_component(self, component_name, attr, attr_value):
        self.update_attribute_entity(
            "Component",
            "component-name",
            component_name,
            attr,
            attr_value)
