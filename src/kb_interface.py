from abc import ABC

from typedb.client import TypeDB
from typedb.client import TypeDBOptions
from typedb.client import SessionType
from typedb.client import TransactionType
from typedb.client import TypeDBClientException

class KbInterface(ABC):
    def __init__(self):
        pass

    def add_objectives(self):
        pass

    def get_objectives(self):
        pass

    def get_function_designs(self, function):
        pass

    def get_ok_function_groundings(self):
        pass

    def get_measured_qas(self):
        pass

    def get_estimated_qas(self):
        pass

    def get_required_qas(self):
        pass

class TypeDBInterface(KbInterface):
    def __del__(self):
        self.client.close()

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

    def write_database_file(self, session_type, transaction_type, query_type, file_path):
        with open(file_path, mode='r') as file:
            query = file.read()

        self.database_query(session_type, transaction_type, query_type, query)

    def load_schema(self, schema_path):
        self.write_database_file(SessionType.SCHEMA, TransactionType.WRITE, 'define', schema_path)

    def load_data(self, data_path, force=False):
        if force:
            query = 'match $e isa entity; delete $e isa entity;'
            self.database_query(SessionType.DATA, TransactionType.WRITE, 'delete', query)
            query = 'match $r isa relation; delete $r isa relation;'
            self.database_query(SessionType.DATA, TransactionType.WRITE, 'delete', query)
            query = 'match $a isa attribute; delete $a isa attribute;'
            self.database_query(SessionType.DATA, TransactionType.WRITE, 'delete', query)
        try:
            self.write_database_file(SessionType.DATA, TransactionType.WRITE, 'insert', data_path)
        except TypeDBClientException as err:
            print("Error in load_data method. This is the exception msg: ", err)

    # TODO: decorator?
    def match_database(query):
        return self.database_query(SessionType.DATA, TransactionType.READ, 'match', query)
    ### Read/write database end

    def get_leaf_functions(self, root_function_name):
        query = f'''
            match
                $f isa Function, has function-name "{root_function_name}";
                (parent-function:$f, child-function:$leaf_function) isa implicit-functional-hierarchy;
                $leaf_function has function-name $fcn;
                $leaf_function has is-leaf-function true;
            get $leaf_function, $function_name;
        '''
        return self.match_database(query)

    def get_ok_function_groundings(self):
        query = '''
            match
                $fg isa FunctionGrounding;
                not {$fg isa FunctionGrounding, has function-grounding-status $fg-status; $fg-status = "error";};
        '''
        return self.match_database(query)
