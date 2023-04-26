import pytest
from typedb_interface import TypeDBInterface


@pytest.fixture
def model_interface():
    function_designs_ordering_funcs = {
        'Execute and control AUV motion': 'function_designs_order_desc'}
    component_ordering_funcs = {'component type': 'component_order_desc'}
    typedb_interface = TypeDBInterface(
        "localhost:1729",
        "test_database",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/empty.tql",  # TODO:better way to handle empty data
        force_database=True,
        force_data=True)
    return typedb_interface


def test_insert_entity(model_interface):
    model_interface.insert_entity('Task', 'task-name', 't1')
    query = f'''
        match $entity isa Task, has task-name "t1";
        get $entity;
    '''
    result = model_interface.match_database(query)
    assert len(result) > 0


def test_delete_entity(model_interface):
    model_interface.insert_entity('Task', 'task-name', 't1')
    model_interface.delete_entity('Task', 'task-name', 't1')
    query = f'''
        match $entity isa Task, has task-name "t1";
        get $entity;
    '''
    result = model_interface.match_database(query)
    assert len(result) == 0
