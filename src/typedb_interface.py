from typedb.client import TypeDB
from typedb.client import TypeDBOptions
from typedb.client import SessionType
from typedb.client import TransactionType
from typedb.client import TypeDBClientException


class TypeDBInterface:
    def __del__(self):
        self.client.close()

    def __init__(self, address, database_name, schema_path, data_path=None,
                 force_database=False, force_data=False):
        self.connect_client(address)
        self.create_database(database_name, force=force_database)
        self.load_schema(schema_path)

        if data_path is not None:
            self.load_data(data_path, force=force_data)

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
                    if query_type == 'match':
                        answer_list = []
                        for answer in query_answer:
                            answer_list.append(answer.map())
                        return answer_list
                    elif query_type == 'match_aggregate':
                        answer = query_answer.get()
                        if answer.is_nan():
                            return None
                        if answer.is_int():
                            return answer.as_int()
                        if answer.is_float():
                            return answer.as_float()

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
        return self.database_query(
            SessionType.DATA, TransactionType.WRITE, 'insert', query)

    # Delete query
    @delete_data_event
    def delete_from_database(self, query):
        return self.database_query(
            SessionType.DATA, TransactionType.WRITE, 'delete', query)

    # TODO: decorator?
    # Match query
    def match_database(self, query):
        options = TypeDBOptions.core()
        options.infer = True
        return self.database_query(
            SessionType.DATA, TransactionType.READ, 'match', query, options)

    def match_aggregate_database(self, query):
        options = TypeDBOptions.core()
        options.infer = True
        return self.database_query(
            SessionType.DATA,
            TransactionType.READ,
            'match_aggregate',
            query,
            options)
    # Read/write database end

    def delete_entity(self, entity, key, key_value):
        query = f'''
            match $entity isa {entity}, has {key} "{key_value}";
            delete $entity isa {entity};
        '''
        return self.delete_from_database(query)

    def insert_entity(self, entity, key, key_value):
        query = f'''
            insert $entity isa {entity}, has {key} "{key_value}";
        '''
        return self.insert_database(query)

    def get_attribute_from_entity_raw(self, entity, key, key_value, attr):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            get $attribute;
        '''
        return self.match_database(query)

    def get_attribute_from_entity(self, entity, key, key_value, attr):
        result = self.get_attribute_from_entity_raw(
            entity, key, key_value, attr)
        return [r.get("attribute").get_value() for r in result]

    def delete_attribute_from_entity(self, entity, key, key_value, attr):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            delete $entity has $attribute;
        '''
        return self.delete_from_database(query)

    def insert_attribute_entity(
     self, entity, key, key_value, attr, attr_value):
        query = f'''
            match $entity isa {entity},
            has {key} "{key_value}";
            insert $entity has {attr} {attr_value};
        '''
        return self.insert_database(query)

    def update_attribute_entity(
     self, entity, key, key_value, attr, attr_value):
        self.delete_attribute_from_entity(
            entity, key, key_value, attr)
        return self.insert_attribute_entity(
            entity, key, key_value, attr, attr_value)
