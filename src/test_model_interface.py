import pytest
from model_interface import ModelInterface


@pytest.fixture
def kb_interface():
    kb_interface = ModelInterface(
        "localhost:1729",
        "pytest_database",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/test_data.tql",  # TODO:better way to handle empty data
        force_database=True,
        force_data=True)
    return kb_interface


def test_request_task(kb_interface):
    kb_interface.request_task('task1')
    is_required = kb_interface.is_task_required('task1')
    assert is_required is True


def test_cancel_task(kb_interface):
    kb_interface.request_task('task1')
    kb_interface.cancel_task('task1')
    is_required = kb_interface.is_task_required('task1')
    assert is_required is False


@pytest.mark.parametrize("task_name, expected_result", [
    ('task_required', True),
    # ('task_required_activated', True),
    ('task_not_required', False),
    ('task_required_empty', False),
])
def test_is_task_required(kb_interface, task_name, expected_result):
    assert kb_interface.is_task_required(task_name) is expected_result


@pytest.mark.parametrize("task_name, expected_result", [
    ('task_required', False),
    ('task_feasible', True),
])
def test_is_task_feasible(kb_interface, task_name, expected_result):
    assert kb_interface.is_task_feasible(task_name) is expected_result

# TODO: is this needed?
# @pytest.mark.parametrize("task_name, expected_result", [
#     ('task_required', True),
#     ('task_required_solved', False),
#     ('task_not_required', False),
#     ('task_required_empty', False),
# ])
# def test_get_tasks_not_solved(kb_interface, task_name, expected_result):
#     unsolved = kb_interface.get_required_tasks_not_solved()
#     assert (task_name in unsolved) is expected_result

#TODO: test_get_required_unsolved_tasks (if it is used)


# def test_has_tasks_not_solved(kb_interface):
#     assert kb_interface.has_required_tasks_not_solved() is True

def test_get_unsolved_functions(kb_interface):
    kb_interface.request_task('task1')
    unsolved_functions = kb_interface.get_unsolved_functions()
    from collections import Counter
    assert Counter(['function1', 'function2']) == Counter(unsolved_functions)


def test_get_unsolved_components(kb_interface):
    unsolved_components = kb_interface.get_unsolved_components()
    assert 'c_required' in unsolved_components


def test_propagate_component_performance(kb_interface):
    kb_interface.propagate_components_performance()
    performance = kb_interface.get_attribute_from_entity(
         'Component',
         'component-name',
         'component1',
         'performance')
    assert performance[0] == 10.0


def test_propagate_function_design_performance(kb_interface):
    kb_interface.propagate_function_designs_performance()
    performance = kb_interface.get_attribute_from_entity(
         'function-design',
         'function-design-name',
         'f3_fd1_c6_c7',
         'performance')
    assert performance[0] == 3.34


def test_propagate_performance(kb_interface):
    kb_interface.propagate_performance()
    c_performance = kb_interface.get_attribute_from_entity(
         'Component',
         'component-name',
         'component1',
         'performance')
    fd_performance = kb_interface.get_attribute_from_entity(
         'function-design',
         'function-design-name',
         'f1_fd1',
         'performance')
    assert c_performance[0] == 10.0 and fd_performance[0] == 10.0


def test_get_measurement_attribute(kb_interface):
    value = 1.0
    measured_value = kb_interface.get_measured_attribute('ea_measurement')
    assert value == measured_value


def test_update_measurement_attribute(kb_interface):
    value = 1.32
    kb_interface.update_measured_attribute('ea1', 1.32)
    measured_value = kb_interface.get_measured_attribute('ea1')
    assert value == measured_value


def test_get_function_design_higher_performance(kb_interface):
    fd = kb_interface.get_function_design_higher_performance('function2')
    assert fd[0].get("fd-name").get_value() == 'f2_fd1_c2_c3'


def test_get_best_function_design(kb_interface):
    fd = kb_interface.get_best_function_design('function2')
    assert fd == 'f2_fd1_c2_c3'


def test_select_function_design(kb_interface):
    kb_interface.select_function_design('f2_fd1_c2_c3', 'true')
    fd_selected = kb_interface.get_attribute_from_entity(
         'function-design',
         'function-design-name',
         'f2_fd1_c2_c3',
         'is-selected')
    kb_interface.select_function_design('f2_fd1_c2_c3', 'false')
    fd_not_selected = kb_interface.get_attribute_from_entity(
         'function-design',
         'function-design-name',
         'f2_fd1_c2_c3',
         'is-selected')
    assert fd_selected[0] is True and fd_not_selected[0] is False


def test_get_component_configuration_higher_performance(kb_interface):
    config = kb_interface.get_component_configuration_higher_performance(
        'component1')
    assert config[0].get("conf-name").get_value() == 'high param'


def test_get_best_component_configuration(kb_interface):
    config = kb_interface.get_best_component_configuration(
        'component1')
    assert config == 'high param'


def test_select_component_configuration(kb_interface):
    kb_interface.select_component_configuration('high param', 'true')
    selected = kb_interface.get_attribute_from_entity(
         'component-configuration',
         'component-configuration-name',
         'high param',
         'is-selected')
    kb_interface.select_component_configuration('high param', 'false')
    not_selected = kb_interface.get_attribute_from_entity(
         'component-configuration',
         'component-configuration-name',
         'high param',
         'is-selected')
    assert selected[0] is True and not_selected[0] is False
# def test_get_required_functions_from_task(kb_interface):
#     result = kb_interface.get_required_functions_from_task('task1')
#     assert all(x in result for x in ['function1', 'function2']) is True
#
#
# def test_get_required_functions(kb_interface):
#     assert False
#
#
# def test_function_required_empty(kb_interface):
#     assert False
