import pytest
from typedb_interface import TypeDBInterface


@pytest.fixture
def kb_interface():
    function_designs_ordering_funcs = {
        'Execute and control AUV motion': 'function_designs_order_desc'}
    component_ordering_funcs = {'component type': 'component_order_desc'}
    kb_interface = TypeDBInterface(
        "localhost:1729",
        "test_database",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/empty.tql",  # TODO:better way to handle empty data
        force_database=True,
        force_data=True)
    return kb_interface


def test_insert_entity(kb_interface):
    kb_interface.insert_entity('Task', 'task-name', 't1')
    query = f'''
        match $entity isa Task, has task-name "t1";
        get $entity;
    '''
    result = kb_interface.match_database(query)
    assert len(result) > 0


def test_delete_entity(kb_interface):
    kb_interface.insert_entity('Task', 'task-name', 't1')
    kb_interface.delete_entity('Task', 'task-name', 't1')
    query = f'''
        match $entity isa Task, has task-name "t1";
        get $entity;
    '''
    result = kb_interface.match_database(query)
    assert len(result) == 0
