# Copyright 2023 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import pytest
from metacontrol_kb.typedb_model_interface import ModelInterface


@pytest.fixture
def kb_interface():
    kb_interface = ModelInterface(
        "localhost:1729",
        "pytest_database",
        "config/schema.tql",
        "test/test_data/test_data.tql",  # TODO:better way to handle empty data
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
    ('task_unfeasible', False),
])
def test_is_task_feasible(kb_interface, task_name, expected_result):
    assert kb_interface.is_task_feasible(task_name) is expected_result


@pytest.mark.parametrize("task_name, expected_result", [
    ('task_required', True),
    ('task_feasible', True),
    ('task_unfeasible', False),
])
def test_is_task_selectable(kb_interface, task_name, expected_result):
    assert kb_interface.is_task_selectable(task_name) is expected_result


def test_get_selectable_tasks(kb_interface):
    result = kb_interface.get_selectable_tasks()
    expected_result = [
        'task_feasible', 'task_required_solved']
    assert ('task_unfeasible' not in result) \
           and all(r in result for r in expected_result)

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

# TODO: test_get_required_unsolved_tasks (if it is used)


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


def test_get_measurement_attribute(kb_interface):
    value = 1.0
    measured_value = kb_interface.get_measured_attribute('ea_measurement')
    assert value == measured_value


def test_update_measurement_attribute(kb_interface):
    value = 1.32
    kb_interface.update_measured_attribute('ea_measurement', 1.32)
    measured_value = kb_interface.get_measured_attribute('ea_measurement')
    assert value == measured_value


def test_get_function_design_higher_performance(kb_interface):
    fd = kb_interface.get_function_design_higher_performance('function2')
    assert fd[0].get("fd-name").get_value() == 'f2_fd1_c2_c3'


def test_get_best_function_design(kb_interface):
    fd = kb_interface.get_best_function_design('function2')
    assert fd == 'f2_fd1_c2_c3'


def test_toogle_function_design_selection(kb_interface):
    kb_interface.toogle_relationship_selection(
        'function-design', 'f2_fd1_c2_c3', True)
    fd_selected = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    kb_interface.toogle_relationship_selection(
        'function-design', 'f2_fd1_c2_c3', False)
    fd_not_selected = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    assert fd_selected[0] is True and fd_not_selected[0] is False


def test_select_function_design(kb_interface):
    kb_interface.select_function_design('function2', 'f2_fd1_c2_c3')
    fd1_selected = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    kb_interface.select_function_design('function2', 'f2_fd2_c4_c5')
    fd1_not_selected = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    fd2_selected = kb_interface.get_attribute_from_entity(
        'function-design',
        'function-design-name',
        'f2_fd2_c4_c5',
        'is-selected')

    assert fd1_selected[0] is True and fd1_not_selected[0] is False \
           and fd2_selected[0] is True


def test_get_component_configuration_higher_performance(kb_interface):
    config = kb_interface.get_component_configuration_higher_performance(
        'component1')
    assert config[0].get("conf-name").get_value() == 'high param'


def test_get_best_component_configuration(kb_interface):
    config = kb_interface.get_best_component_configuration(
        'component1')
    assert config == 'high param'


def test_select_component_configuration(kb_interface):
    kb_interface.select_component_configuration('component1', 'high param')
    high_selected = kb_interface.get_attribute_from_entity(
        'component-configuration',
        'component-configuration-name',
        'high param',
        'is-selected')
    kb_interface.select_component_configuration('component1', 'low param')
    high_not_selected = kb_interface.get_attribute_from_entity(
        'component-configuration',
        'component-configuration-name',
        'high param',
        'is-selected')
    low_selected = kb_interface.get_attribute_from_entity(
        'component-configuration',
        'component-configuration-name',
        'low param',
        'is-selected')
    assert high_selected[0] is True and high_not_selected[0] is False and \
           low_selected[0] is True
