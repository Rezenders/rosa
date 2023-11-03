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
from datetime import datetime


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
    assert fd[0].get("fd-name").get('value') == 'f2_fd1_c2_c3'


def test_get_best_function_design(kb_interface):
    fd = kb_interface.get_best_function_design('function2')
    assert fd == 'f2_fd1_c2_c3'


def test_toogle_function_design_selection(kb_interface):
    kb_interface.toogle_thing_selection(
        'function-design', 'f2_fd1_c2_c3', True)
    fd_selected = kb_interface.get_attribute_from_thing(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    kb_interface.toogle_thing_selection(
        'function-design', 'f2_fd1_c2_c3', False)
    fd_not_selected = kb_interface.get_attribute_from_thing(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    assert fd_selected[0] is True and fd_not_selected[0] is False


def test_select_function_design(kb_interface):
    kb_interface.select_function_design('function2', 'f2_fd1_c2_c3')
    fd1_selected = kb_interface.get_attribute_from_thing(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    kb_interface.select_function_design('function2', 'f2_fd2_c4_c5')
    fd1_not_selected = kb_interface.get_attribute_from_thing(
        'function-design',
        'function-design-name',
        'f2_fd1_c2_c3',
        'is-selected')
    fd2_selected = kb_interface.get_attribute_from_thing(
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
    high_selected = kb_interface.get_attribute_from_thing(
        'component-configuration',
        'component-configuration-name',
        'high param',
        'is-selected')
    kb_interface.select_component_configuration('component1', 'low param')
    high_not_selected = kb_interface.get_attribute_from_thing(
        'component-configuration',
        'component-configuration-name',
        'high param',
        'is-selected')
    low_selected = kb_interface.get_attribute_from_thing(
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
   (['component2', 'component3'], ['component4', 'component5'], []),
   ([], [], ['low param']),
   ([], [], []),
])
def test_create_reconfiguration_plan(
   kb_interface, c_activate, c_deactivate, c_config):
    result, start_time = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    query = "match ("
    end_query = ""
    if len(c_activate) == 0 and len(c_deactivate) == 0 and len(c_config) == 0:
        assert result is True and start_time is None
    else:
        if len(c_activate) > 0:
            query += "architectural-adaptation:$ca"
            _match_query, _prefix_list = kb_interface.create_match_query(
                [('Component', 'component-name', c) for c in c_activate],
                'ca_')
            end_query += kb_interface.create_relationship_insert_query(
                'component-activation',
                {'component': _prefix_list},
                prefix='ca'
            )
            end_query += _match_query

        if len(c_deactivate) > 0:
            if query != "match (":
                query += ','
            query += "architectural-adaptation:$cd"
            _match_query, _prefix_list = kb_interface.create_match_query(
                [('Component', 'component-name', c) for c in c_deactivate],
                'cd_')
            end_query += kb_interface.create_relationship_insert_query(
                'component-deactivation',
                {'component': _prefix_list},
                prefix='cd'
            )
            end_query += _match_query

        if len(c_config) > 0:
            if query != "match (":
                query += ','
            query += "parameter-adaptation:$pa"
            _match_query, _prefix_list = kb_interface.create_match_query(
                [('component-configuration', 'component-configuration-name', c)
                 for c in c_config], 'cc_')
            end_query += kb_interface.create_relationship_insert_query(
                'parameter-adaptation',
                {'component-configuration': _prefix_list},
                prefix='cc'
            )
            end_query += _match_query

        query += f"""
            ) isa reconfiguration-plan,
                has start-time {start_time.isoformat(timespec='milliseconds')};
        """
        query += end_query
        query_result = kb_interface.match_database(query)
        assert len(query_result) > 0


@pytest.mark.parametrize("selected_fd, selected_config, exp_c_activate, exp_c_deactivate, exp_c_config ", [
    ([('f_reconfigure_fd', 'fd_reconfig_1')], [('component_reconfig_3', 'cp_reconfig_1')], [], [], []),
    ([('f_reconfigure_fd', 'fd_reconfig_2')], [('component_reconfig_3', 'cp_reconfig_1')], ['component_reconfig_2'], ['component_reconfig_1'], []),
    ([('f_reconfigure_fd', 'fd_reconfig_1')], [('component_reconfig_3', 'cp_reconfig_2')], [], [], ['cp_reconfig_2']),
    ([('f_reconfigure_fd', 'fd_reconfig_2')], [('component_reconfig_3', 'cp_reconfig_2')], ['component_reconfig_2'], ['component_reconfig_1'], ['cp_reconfig_2']),
])
def test_select_configuration(
        kb_interface,
        selected_fd,
        selected_config,
        exp_c_activate,
        exp_c_deactivate,
        exp_c_config
     ):
    result, start_time = kb_interface.select_configuration(
        selected_fd, selected_config)

    query = "match $rp "
    end_query = ""
    if len(exp_c_activate) > 0:
        if query == "match $rp ":
            query += '('
        query += "architectural-adaptation:$ca"
        _match_query, _prefix_list = kb_interface.create_match_query(
            [('Component', 'component-name', c) for c in exp_c_activate], 'ca')
        end_query += kb_interface.create_relationship_insert_query(
            'component-activation',
            {'component': _prefix_list},
            prefix='ca'
        )
        end_query += _match_query
    if len(exp_c_deactivate) > 0:
        if query == "match $rp ":
            query += '('
        elif len(query) > len("match $rp ("):
            query += ','
        query += "architectural-adaptation:$cd"
        _match_query, _prefix_list = kb_interface.create_match_query(
            [('Component', 'component-name', c) for c in exp_c_deactivate], 'cd')
        end_query += kb_interface.create_relationship_insert_query(
            'component-deactivation',
            {'component': _prefix_list},
            prefix='cd'
        )
        end_query += _match_query
    if len(exp_c_config) > 0:
        if query == "match $rp ":
            query += '('
        elif len(query) > len("match $rp ("):
            query += ','
        query += "parameter-adaptation:$pa"
        _match_query, _prefix_list = kb_interface.create_match_query(
            [('component-configuration', 'component-configuration-name', c)
             for c in exp_c_config], 'cc_')
        end_query += kb_interface.create_relationship_insert_query(
            'parameter-adaptation',
            {'component-configuration': _prefix_list},
            prefix='cc'
        )
        end_query += _match_query

    if query != "match $rp ":
        query += ')'

    query += " isa reconfiguration-plan, has start-time {};".format(
        start_time.isoformat(timespec='milliseconds'))
    query += end_query
    query_result = kb_interface.match_database(query)

    right_fd_selected = True
    for fd in selected_fd:
        _fd = kb_interface.get_relationship_with_attribute(
            'Function',
            fd[0],
            'function-design',
            'is-selected',
            True
        )
        right_fd_selected = (fd[1] == _fd[0])
    right_conf_selected = True
    for config in selected_config:
        _config = kb_interface.get_relationship_with_attribute(
            'Component',
            config[0],
            'component-configuration',
            'is-selected',
            True
        )
        right_conf_selected = (config[1] == _config[0])
    assert len(query_result) > 0 and right_fd_selected and right_conf_selected


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


def test_get_latest_reconfiguration_plan_time(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    lastest_plan = kb_interface.get_latest_reconfiguration_plan_time()
    assert lastest_plan == start_time_2


def test_get_latest_reconfiguration_plan_time_no_rp(kb_interface):
    lastest_plan = kb_interface.get_latest_reconfiguration_plan_time()
    assert lastest_plan is False


def test_get_reconfiguration_plan(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    reconfig_plan = kb_interface.get_reconfiguration_plan(start_time)
    assert sorted(c_activate) == sorted(reconfig_plan['c_activate']) and \
        sorted(c_deactivate) == sorted(reconfig_plan['c_deactivate']) and \
        sorted(c_config) == sorted(reconfig_plan['c_config'])


def test_get_latest_reconfiguration_plan(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    reconfig_plan = kb_interface.get_latest_reconfiguration_plan()
    assert reconfig_plan['start-time'] == start_time_2 and \
        sorted(c_activate) == sorted(reconfig_plan['c_activate']) and \
        sorted(c_deactivate) == sorted(reconfig_plan['c_deactivate']) and \
        sorted(c_config) == sorted(reconfig_plan['c_config'])


def test_update_reconfiguration_plan_result(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    kb_interface.update_reconfiguration_plan_result(start_time, 'completed')
    query = f'''
        match $rp isa reconfiguration-plan,
            has start-time {start_time.isoformat(timespec='milliseconds')},
            has end-time $end-time,
            has result $result;
            get $end-time, $result;
    '''
    query_result = kb_interface.match_database(query)
    result = [r.get('result').get('value') for r in query_result]
    assert len(result) > 0 and result[0] == 'completed'


def test_get_latest_completed_reconfiguration_plan_time(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    kb_interface.update_reconfiguration_plan_result(start_time_1, 'completed')

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    kb_interface.update_reconfiguration_plan_result(start_time_2, 'completed')
    end_time = kb_interface.get_latest_completed_reconfiguration_plan_time()
    assert end_time is not False and end_time is not None \
        and type(end_time) is datetime


def test_get_latest_pending_reconfiguration_plan_time(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    kb_interface.update_reconfiguration_plan_result(start_time_2, 'completed')
    r_start_time = kb_interface.get_latest_pending_reconfiguration_plan_time()
    assert r_start_time is not False and r_start_time is not None \
        and type(r_start_time) is datetime and r_start_time == start_time_1


def test_get_outdated_reconfiguration_plans(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)
    kb_interface.update_reconfiguration_plan_result(start_time_2, 'completed')

    times = kb_interface.get_outdated_reconfiguration_plans()

    assert start_time_1 in times


def test_get_reconfiguration_plan_result(kb_interface):
    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    kb_interface.update_reconfiguration_plan_result(start_time, 'completed')
    result = kb_interface.get_reconfiguration_plan_result(start_time)
    assert result == 'completed'


def test_update_outdated_reconfiguration_plans_result(kb_interface):
    kb_interface.update_outdated_reconfiguration_plans_result()

    c_activate = ['component2', 'component3']
    c_deactivate = ['component4', 'component5']
    c_config = ['low param']
    r, start_time_1 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component3']
    c_deactivate = ['component5']
    c_config = ['low param']
    r2, start_time_2 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    c_activate = ['component2']
    c_deactivate = ['component4']
    c_config = ['high param']
    r3, start_time_3 = kb_interface.create_reconfiguration_plan(
        c_activate, c_deactivate, c_config)

    kb_interface.update_reconfiguration_plan_result(start_time_2, 'completed')
    kb_interface.update_outdated_reconfiguration_plans_result()

    result1 = kb_interface.get_reconfiguration_plan_result(start_time_1)
    result2 = kb_interface.get_reconfiguration_plan_result(start_time_2)
    result3 = kb_interface.get_reconfiguration_plan_result(start_time_3)

    assert result1 == 'abandoned' and result2 == 'completed' \
           and result3 == 'abandoned'


def test_get_selectable_fds(kb_interface):
    result = kb_interface.get_selectable_fds('f_fd_feasible_unfeasible')
    expected_result = ['f_fd_feasible']
    assert all(r in result for r in expected_result)


def test_get_selectable_c_configs(kb_interface):
    result = kb_interface.get_selectable_c_configs('c_cc_feasible_unfeasible')
    expected_result = ['c_cc_feasible']
    assert all(r in result for r in expected_result)
