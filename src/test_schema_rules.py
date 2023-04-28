import pytest
from typedb_interface import TypeDBInterface


@pytest.fixture
def kb_interface():
    kb_interface = TypeDBInterface(
        "localhost:1729",
        "test_database",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/test_data.tql",  # TODO:better way to handle empty data
        force_database=True,
        force_data=True)
    return kb_interface


@pytest.mark.parametrize("components, fd_name, fd_selected, fd_status", [
    ([('component2', 'failure')], 'f2_fd1_c2_c3', 'false', 'unfeasible'),
    ([('component2', 'unfeasible')], 'f2_fd1_c2_c3', 'false', 'unfeasible'),
    ([('component2', 'unfeasible')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component3', 'configuration error')], 'f2_fd1_c2_c3', 'true', 'implict configuration error'),
    ([('component3', 'configuration error')], 'f2_fd1_c2_c3', 'false', 'ok'),
    ([('component2', 'failure'), ('component3', 'configuration error')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component2', 'configuration error'), ('component3', 'ok')], 'f2_fd1_c2_c3', 'true', 'implict configuration error'),
    ([('component2', 'failure'), ('component3', 'ok')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component2', 'ok'), ('component3', 'ok')], 'f2_fd1_c2_c3', 'true', 'ok'),
])
def test_function_design_status_inference(
     kb_interface, components, fd_name, fd_selected, fd_status):

    for component in components:
        kb_interface.update_attribute_entity(
            'Component',
            'component-name',
            component[0],
            'component-status',
            "'{}'".format(component[1]))
    kb_interface.update_attribute_entity(
        'function-design',
        'function-design-name',
        fd_name,
        'is-selected',
        fd_selected)
    fd_status_inferred = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        fd_name,
        'function-design-status')
    assert all(x in fd_status_inferred for x in [fd_status]) is True


@pytest.mark.parametrize("fds, f_name, f_required, f_status", [
    ([], 'function_no_fd', 'false', 'unfeasible'),
    ([], 'function1', 'true', 'unsolved'),
    ([], 'function1', 'false', 'ok'),
    ([('f1_fd1', 'unfeasible', 'false')], 'function1', 'false', 'unfeasible'),
    ([('f1_fd1', 'unfeasible', 'false')], 'function1', 'true', 'unfeasible'),
    ([('f2_fd1_c2_c3', 'unfeasible', 'true')], 'function2', 'true', 'configuration error'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'true')], 'function2', 'true', 'implicit configuration error'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'false')], 'function2', 'false', 'ok'),
    ([('f2_fd1_c2_c3', 'ok', 'true'), ('f2_fd2_c4_c5', 'ok', 'false')], 'function2', 'true', 'ok'),
])
def test_function_status_inference(
     kb_interface, fds, f_name, f_required, f_status):
    for fd in fds:
        kb_interface.update_attribute_entity(
            'function-design',
            'function-design-name',
            fd[0],
            'function-design-status',
            "'{}'".format(fd[1]))
        if fd[2] == 'true':
            kb_interface.update_attribute_entity(
                'function-design',
                'function-design-name',
                fd[0],
                'is-selected',
                'true')
    kb_interface.update_attribute_entity(
        'Function',
        'function-name',
        f_name,
        'is-required',
        f_required)
    f_status_inferred = kb_interface.get_attribute_from_entity(
        'Function',
        'function-name',
        f_name,
        'function-status')
    print(f_status_inferred)
    assert all(x in f_status_inferred for x in [f_status]) is True
