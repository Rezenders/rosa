from abc import ABC

from typedb.client import TypeDB
from typedb.client import TypeDBOptions
from typedb.client import SessionType
from typedb.client import TransactionType
from typedb.client import TypeDBClientException


class TypeDBInterface():
    def __del__(self):
        self.client.close()

    def __init__(self, address, database_name, schema_path, data_path,
                    force_database=False, force_data=False,
                    function_designs_ordering_funcs=dict(),
                    default_function_design_ordering_func='function_designs_order_desc',
                    component_ordering_funcs=dict(),
                    default_component_ordering_func='component_order_asc'):
        self.connect_client("localhost:1729")
        self.create_database("pipeline_inspection", force=force_database)
        self.load_schema("../typeDB/schema/schema.tql")
        self.load_data("../typeDB/data/example_search_pipeline.tql", force=force_data)

        self.function_designs_ordering_funcs = function_designs_ordering_funcs
        self.default_function_design_ordering_func = default_function_design_ordering_func

        self.component_ordering_funcs = component_ordering_funcs
        self.default_component_ordering_func = default_component_ordering_func

    def connect_client(self, address, parallelisation=2):
        self.client = TypeDB.core_client(address=address, parallelisation=parallelisation)

    def create_database(self, database_name, force=False):
        if self.client.databases().contains(database_name) and force:
            self.client.databases().get(database_name).delete()

        if not self.client.databases().contains(database_name):
            self.database = self.client.databases().create(database_name)
            self.database_name = database_name
        else:
            self.database_name = database_name
            print("The database with the name ", database_name, " already exists. Ignoring create_database request.")

    def create_session(self, database_name, session_type, options=TypeDBOptions.core()):
        return self.client.session(database_name, session_type, options)

    #### Read/write database
    # Generic query method
    def database_query(self, session_type, transaction_type, query_type, query, options=TypeDBOptions.core()):
        with self.create_session(self.database_name, session_type) as session:
            options.infer = True
            with session.transaction(transaction_type, options) as transaction:
                transaction_query_function = getattr(transaction.query(), query_type)
                answer_iterator = transaction_query_function(query)
                if transaction_type == TransactionType.WRITE:
                    transaction.commit()
                elif transaction_type == TransactionType.READ:
                    answer_list = []
                    for answer in answer_iterator:
                        answer_list.append(answer.map())
                    return answer_list

    # Load schema or data from file
    def write_database_file(self, session_type, transaction_type, query_type, file_path):
        with open(file_path, mode='r') as file:
            query = file.read()

        self.database_query(session_type, transaction_type, query_type, query)

    # Load schema from file
    def load_schema(self, schema_path):
        self.write_database_file(SessionType.SCHEMA, TransactionType.WRITE, 'define', schema_path)

    # Load data from file
    def load_data(self, data_path, force=False):
        if force:
            self.delete_from_database('match $e isa entity; delete $e isa entity;')
            self.delete_from_database('match $r isa relation; delete $r isa relation;')
            self.delete_from_database('match $a isa attribute; delete $a isa attribute;')
        try:
            self.write_database_file(SessionType.DATA, TransactionType.WRITE, 'insert', data_path)
        except TypeDBClientException as err:
            print("Error in load_data method. This is the exception msg: ", err)

    ### Events begining
    def insert_data_event(func):
        def inner(*args, **kwargs):
            print('Data has been inserted!') #TODO: Do something else
            return func(*args, **kwargs)
        return inner

    def delete_data_event(func):
        def inner(*args, **kwargs):
            print('Data has been deleted!') #TODO: Do something else
            return func(*args, **kwargs)
        return inner
    ### Events end

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

    ### Read/write database end

    ### Queries begining
    def get_leaf_functions_from_task(self, task_name):
        query = f'''
            match
                $task isa Task, has task-name "{task_name}";
                (task:$task, required-function:$function) isa task-requirement;
                $function isa Function, has function-name $function_name;
                $leaf_function_name isa function-name;
                {{$function has is-leaf-function true, has function-name $leaf_function_name;}} or
                {{(parent-function:$function, child-function:$fc) isa implicit-functional-hierarchy;
                    $fc has is-leaf-function true, has function-name $leaf_function_name; }};
                get $function_name, $leaf_function_name;
        '''
        return self.match_database(query)

    def get_required_functions_from_task(self, task_name):
        query = f'''
            match
                $t isa Task, has task-name "{task_name}";
                (task:$t, required-function:$function) isa task-requirement;
                $function isa Function, has function-name $function_name;
                get $function_name;
        '''
        return self.match_database(query)

    #TODO: standard for variable names in queries
    # TODO: remove unecessary variables from query
    def get_leaf_functions(self, root_function_name):
        query = f'''
            match
                $f isa Function, has function-name "{root_function_name}";
                (parent-function:$f, child-function:$leaf_function) isa implicit-functional-hierarchy;
                $leaf_function has function-name $function_name;
                $leaf_function has is-leaf-function true;
            get $leaf_function, $function_name;
        '''
        return self.match_database(query)

    # fds ordered desc single qa
    def function_designs_order_desc(self, function_name):
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function:$f, required-component:$c) isa function-design;
                $fd isa function-design, has function-design-name $function_design_name;
                (function-design:$fd, qa:$eqa) isa estimated-qa;
                $eqa isa EstimatedQualityAttribute, has qa-type $qa-type, has qa-value $qa-value;
                get $function_design_name, $qa-type, $qa-value;
                sort $qa-value desc;
        '''
        return self.match_database(query)

    # fds ordered asc single qa
    def function_designs_order_asc(self, function_name):
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function:$f, required-component:$c) isa function-design;
                $fd isa function-design, has function-design-name $function_design_name;
                (function-design:$fd, qa:$eqa) isa estimated-qa;
                $eqa isa EstimatedQualityAttribute, has qa-type $qa-type, has qa-value $qa-value;
                get $function_design_name, $qa-type, $qa-value;
                sort $qa-value asc;
        '''
        return self.match_database(query)

    #get fd with higher estimated qa (only considering 1 qa)
    def get_function_designs_ordered(self, function_name):
        if function_name in self.function_designs_ordering_funcs:
            return getattr(self,self.function_designs_ordering_funcs[function_name])(function_name)
        else:
            return getattr(self, self.default_function_design_ordering_func)(function_name)

    # get components from fd (only dealing with Components, not ComponentType)
    # TODO: get component-executor
    def get_components_from_function_design(self, function_design_name):
        query = f'''
            match
                $fd (function:$f, required-component:$component) isa function-design,  has function-design-name "{function_design_name}";
                $component isa Component, has component-name $component_name;
                get $component_name;
        '''
        return self.match_database(query)

    def get_component_types_from_function_design(self, function_design_name):
        query = f'''
            match
                $fd (function:$f, required-component:$component) isa function-design,  has function-design-name "{function_design_name}";
                $component isa ComponentType, has component-type $component_type;
                get $component_type;
        '''
        return self.match_database(query)

    def component_order_asc(self, component_type):
        query = f'''
            match
                $ct isa ComponentType, has component-type "{component_type}";
                $cd (componentType:$ct, component:$component) isa component-design, has component-design-priority $priority;
                $component isa Component, has component-name $component_name;
                get $component_name, $priority;
                sort $priority asc;
        '''
        return self.match_database(query)

    def component_order_desc(self, component_type):
        query = f'''
            match
                $ct isa ComponentType, has component-type "{component_type}";
                $cd (componentType:$ct, component:$component) isa component-design, has component-design-priority $priority;
                $component isa Component, has component-name $component_name;
                get $component_name, $priority;
                sort $priority desc;
        '''
        return self.match_database(query)

    def get_components_from_component_type(self, component_type):
        if component_type in self.component_ordering_funcs:
            return getattr(self,self.component_ordering_funcs[component_type])(component_type)
        else:
            return getattr(self, self.default_component_ordering_func)(component_type)

    def get_component_executor(self, component_name):
        query = f'''
            match
                $c isa Component, has component-name "{component_name}", has component-executor $component_executor;
                get $component_executor;
        '''
        return self.match_database(query)
