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


@pytest.fixture(scope="module")
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
    assert fd[0].get("fd-name").get('value') == 'f2_fd1_c2_c3'


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
    assert config[0].get("conf-name").get('value') == 'high param'


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

@pytest.mark.parametrize("c_activate, c_deactivate, c_config", [
    (['component2', 'component3'], ['component4', 'component5'], ['low param']),
    ([], ['component4', 'component5'], ['low param']),
    (['component2', 'component3'], [], ['low param']),
    ([], [], ['low param']),
    (['component2', 'component3'], ['component4', 'component5'], []),
])
def test_create_reconfiguration_plan(
   kb_interface, c_activate, c_deactivate, c_config):
    result, start_time = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    query = "match ("
    end_query = ""
    if len(c_activate) > 0:
        query += "architectural-adaptation:$ca"
        components = "("
        c_name = ""
        counter = 0
        for c in c_activate:
            if components != "(":
                components += ","
            components += "component: $ca_{}".format(counter)
            c_name += "$ca_{0} isa Component, has component-name '{1}';".format(counter, c)
            counter += 1
        components += ")"
        end_query += " $ca {} isa component-activation;".format(components)
        end_query += c_name

    if len(c_deactivate) > 0:
        if query != "match (":
            query += ','
        query += "architectural-adaptation:$cd"
        components = "("
        c_name = ""
        counter = 0
        for c in c_deactivate:
            if components != "(":
                components += ","
            components += "component: $cd_{}".format(counter)
            c_name += "$cd_{0} isa Component, has component-name '{1}';" \
                .format(counter, c)
            counter += 1
        components += ")"
        end_query += " $cd {} isa component-deactivation;".format(components)
        end_query += c_name

    if len(c_config) > 0:
        if query != "match (":
            query += ','
        query += "parameter-adaptation:$pa"
        configs = "("
        c_name = ""
        counter = 0
        for c in c_config:
            if configs != "(":
                configs += ","
            configs += "component-configuration: $cc_{}".format(counter)
            c_name += "$cc_{0} isa component-configuration, \
                has component-configuration-name '{1}';".format(counter, c)
            counter += 1
        configs += ")"
        end_query += " $pa {} isa parameter-adaptation;".format(configs)
        end_query += c_name
    query += ")"

    query += f""" isa reconfiguration-plan, has start-time {start_time};"""
    query += end_query
    query_result = kb_interface.match_database(query)

    assert len(query_result) > 0



    query = f"""
        match
        $rp (architectural-adaptation:$aa, parameter-adaptation:$pa)
            isa reconfiguration-plan,
            has start-time {start_time};
    """
    query_result = kb_interface.match_database(query)

    assert len(query_result) > 0


@pytest.mark.parametrize("thing, status, exp", [
    ('Task', 'feasible', 'task_feasible'),
    ('Task', 'unfeasible', 'task_unfeasible'),
    ('Function', 'unsolved', 'f_unsolved'),
    ('Component', 'unsolved', 'c_unsolved'),
    ('function-design', 'unsolved', 'fd_unsolved'),
])
def test_get_instances_of_thing_with_status(kb_interface, thing, status, exp):
    result = kb_interface.get_instances_of_thing_with_status(thing, status)
    assert exp in result


@pytest.mark.parametrize("thing, exp", [
    ('Function', 'f_always_improve'),
    ('Component', 'c_always_improve'),
])
def test_get_instances_thing_always_improve(kb_interface, thing, exp):
    result = kb_interface.get_instances_thing_always_improve(thing)
    assert exp in result


def test_get_adaptable_functions(kb_interface):
    expected_result = ['f_unsolved', 'f_always_improve']
    result = kb_interface.get_adaptable_functions()
    assert all(r in result for r in expected_result)


def test_get_adaptable_component(kb_interface):
    expected_result = ['c_unsolved', 'c_always_improve']
    result = kb_interface.get_adaptable_components()
    print(result)
    assert all(r in result for r in expected_result)


def test_get_components_in_function_design(kb_interface):
    result = kb_interface.get_components_in_function_design('f2_fd1_c2_c3')
    expected_result = ['component2', 'component3']
    assert all(r in result for r in expected_result)
