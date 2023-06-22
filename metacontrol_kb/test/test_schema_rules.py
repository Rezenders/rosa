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
from ros_typedb.typedb_interface import TypeDBInterface


@pytest.fixture
def kb_interface():
    kb_interface = TypeDBInterface(
        "localhost:1729",
        "pytest_database",
        "../typeDB/schema/uio_schema.tql",
        "../typeDB/data/test_data.tql",  # TODO:better way to handle empty data
        force_database=True,
        force_data=True)
    return kb_interface


@pytest.mark.parametrize("att_name, att_value, config_name, constrainment_status", [
    ('ea1', '2.5', 'high param', 'violated'),
    ('ea1', '2.5', 'low param', 'satisfied'),
    ('ea1', '', 'low param', 'not evaluated'),
])
def test_constrainment_status_inference(
        kb_interface, att_name, att_value, config_name, constrainment_status):

    if att_value != '':
        kb_interface.update_attribute_entity(
            'Attribute',
            'attribute-name',
            att_name,
            'attribute-measurement',
            att_value)
    query = f'''
        match
            $ea isa EnvironmentalAttribute, has attribute-name "{att_name}";
            $config isa component-configuration, has component-configuration-name "{config_name}";
            (constraint: $ea, constrained: $config) isa constrainment, has constrainment-status $status;
            get $status;
    '''
    query_result = kb_interface.match_database(query)
    inferred_status = [
        status.get("status").get_value() for status in query_result]
    assert inferred_status[0] == constrainment_status


@pytest.mark.parametrize("att_name, att_value, config_name, config_status", [
    ('ea1', '2.5', 'high param', 'unfeasible'),
    ('ea1', '2.5', 'low param', 'feasible'),
    ('ea1', '', 'low param', 'feasible'),
])
def test_component_configuration_status_inference(
        kb_interface, att_name, att_value, config_name, config_status):
    if att_value != '':
        kb_interface.update_attribute_entity(
            'Attribute',
            'attribute-name',
            att_name,
            'attribute-measurement',
            att_value)
    query = f'''
        match
            $config isa component-configuration, has component-configuration-name "{config_name}", has component-configuration-status $status;
            get $status;
    '''
    query_result = kb_interface.match_database(query)
    inferred_status = [
        status.get("status").get_value() for status in query_result]
    assert inferred_status[0] == config_status


@pytest.mark.parametrize("configurations, c_name, c_required, c_active, c_status", [
    ([('low param', 'unfeasible', 'true'), ('high param', 'unfeasible', 'false')], 'component1', 'false', 'false', 'unfeasible'),
    ([('low param', 'unfeasible', 'true'), ('high param', 'feasible', 'false')], 'component1', 'true', 'false', 'configuration error'),
    ([('low param', 'feasible', 'false'), ('high param', 'feasible', 'false')], 'component1', 'true', 'false', 'unsolved'),
    ([('low param', 'feasible', 'true'), ('high param', 'feasible', 'false')], 'component1', 'true', 'false', 'unsolved'),
    ([('low param', 'feasible', 'true'), ('high param', 'unfeasible', 'false')], 'component1', 'true', 'false', 'unsolved'),
    ([], 'component1', 'true', 'false', 'unsolved'),
    ([('low param', 'feasible', 'true'), ('high param', 'feasible', 'false')], 'component1', 'false', 'false', 'feasible'),
    ([('low param', 'feasible', 'false'), ('high param', 'unfeasible', 'false')], 'component1', 'false', 'false', 'feasible'),
    ([], 'component1', 'false', 'false', 'feasible'),
    ([('low param', 'feasible', 'true'), ('high param', 'feasible', 'false')], 'component1', 'true', 'true', 'solved'),
    ([('low param', 'feasible', 'true'), ('high param', 'unfeasible', 'false')], 'component1', 'true', 'true', 'solved'),
])
def test_component_status_inference(
        kb_interface, configurations, c_name, c_required, c_active, c_status):
    for config in configurations:
        kb_interface.update_attribute_entity(
            'component-configuration',
            'component-configuration-name',
            config[0],
            'component-configuration-status',
            "'{}'".format(config[1]))
        if config[2] == 'true':
            kb_interface.update_attribute_entity(
                'component-configuration',
                'component-configuration-name',
                config[0],
                'is-selected',
                'true')
    kb_interface.update_attribute_entity(
        'Component',
        'component-name',
        c_name,
        'is-required',
        c_required)
    kb_interface.update_attribute_entity(
        'Component',
        'component-name',
        c_name,
        'is-active',
        c_active)
    c_status_inferred = kb_interface.get_attribute_from_entity(
        'Component',
        'component-name',
        c_name,
        'component-status')
    print(c_status_inferred)
    assert all(x in c_status_inferred for x in [c_status]) is True


@pytest.mark.parametrize("components, fd_name, fd_selected, fd_status", [
    ([('component2', 'failure')], 'f2_fd1_c2_c3', 'false', 'unfeasible'),
    ([('component2', 'failure'), ('component3', 'feasible')], 'f2_fd1_c2_c3', 'false', 'unfeasible'),
    ([('component2', 'failure'), ('component3', 'solved')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component2', 'failure'), ('component3', 'configuration error')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component2', 'unfeasible')], 'f2_fd1_c2_c3', 'false', 'unfeasible'),
    ([('component2', 'unfeasible')], 'f2_fd1_c2_c3', 'true', 'unfeasible'),
    ([('component2', 'solved'), ('component3', 'configuration error')], 'f2_fd1_c2_c3', 'true', 'implicit configuration error'),
    ([('component2', 'configuration error'), ('component3', 'feasible')], 'f2_fd1_c2_c3', 'true', 'implicit configuration error'),
    ([('component2', 'unsolved'), ('component3', 'unsolved')], 'f2_fd1_c2_c3', 'true', 'unsolved'),
    ([('component2', 'feasible'), ('component3', 'feasible')], 'f2_fd1_c2_c3', 'true', 'unsolved'),
    ([('component2', 'solved'), ('component3', 'unsolved')], 'f2_fd1_c2_c3', 'true', 'unsolved'),
    ([('component2', 'solved'), ('component3', 'feasible')], 'f2_fd1_c2_c3', 'true', 'unsolved'),
    ([], 'f2_fd1_c2_c3', 'true', 'unsolved'),
    ([('component2', 'feasible'), ('component3', 'configuration error')], 'f2_fd1_c2_c3', 'false', 'feasible'),
    ([('component2', 'feasible'), ('component3', 'feasible')], 'f2_fd1_c2_c3', 'false', 'feasible'),
    ([], 'f2_fd1_c2_c3', 'false', 'feasible'),
    ([('component2', 'solved'), ('component3', 'solved')], 'f2_fd1_c2_c3', 'true', 'solved'),
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
        if component[1] == 'solved':
            kb_interface.update_attribute_entity(
                'Component',
                'component-name',
                component[0],
                'is-active',
                'true')
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
    ([('f1_fd1', 'unfeasible', 'false')], 'function1', 'false', 'unfeasible'),
    ([('f1_fd1', 'unfeasible', 'false')], 'function1', 'true', 'unfeasible'),
    ([('f2_fd1_c2_c3', 'unfeasible', 'true'), ('f2_fd2_c4_c5', 'unfeasible', 'false')], 'function2', 'true', 'unfeasible'),
    ([('f2_fd1_c2_c3', 'unfeasible', 'true')], 'function2', 'true', 'configuration error'),
    ([('f2_fd1_c2_c3', 'unfeasible', 'true'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'true', 'configuration error'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'true')], 'function2', 'true', 'implicit configuration error'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'true'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'true', 'implicit configuration error'),
    ([], 'function1', 'true', 'unsolved'),
    ([('f2_fd1_c2_c3', 'feasible', 'false'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'true', 'unsolved'),
    ([('f2_fd1_c2_c3', 'feasible', 'false'), ('f2_fd2_c4_c5', 'unfeasible', 'false')], 'function2', 'true', 'unsolved'),
    ([('f2_fd1_c2_c3', 'unsolved', 'true'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'true', 'unsolved'),
    ([('f2_fd1_c2_c3', 'solved', 'true'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'true', 'solved'),
    ([('f2_fd1_c2_c3', 'solved', 'true'), ('f2_fd2_c4_c5', 'unfeasible', 'false')], 'function2', 'true', 'solved'),
    ([], 'function1', 'false', 'feasible'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'false')], 'function2', 'false', 'feasible'),
    ([('f2_fd1_c2_c3', 'implicit configuration error', 'false'), ('f2_fd2_c4_c5', 'unfeasible', 'false')], 'function2', 'false', 'feasible'),
    ([('f2_fd1_c2_c3', 'feasible', 'false'), ('f2_fd2_c4_c5', 'unfeasible', 'false')], 'function2', 'false', 'feasible'),
    ([('f2_fd1_c2_c3', 'feasible', 'false'), ('f2_fd2_c4_c5', 'feasible', 'false')], 'function2', 'false', 'feasible'),
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


@pytest.mark.parametrize("functions, tr_name, tr_required, tr_status", [
    ([('function1', 'unfeasible')], 'task1 requirement', 'false', 'unfeasible'),
    ([('function1', 'unsolved'), ('function2', 'unfeasible')], 'task1 requirement', 'true', 'unfeasible'),
    ([('function1', 'configuration error'), ('function2', 'unfeasible')], 'task1 requirement', 'true', 'unfeasible'),
    ([('function1', 'configuration error')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'configuration error'), ('function2', 'feasible')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'configuration error'), ('function2', 'implicit configuration error')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'implicit configuration error')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'configuration error'), ('function2', 'solved')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'configuration error'), ('function2', 'unsolved')], 'task1 requirement', 'true', 'implicit configuration error'),
    ([('function1', 'unsolved')], 'task1 requirement', 'true', 'unsolved'),
    ([('function1', 'unsolved'), ('function2', 'solved')], 'task1 requirement', 'true', 'unsolved'),
    ([('function1', 'unsolved'), ('function2', 'unsolved')], 'task1 requirement', 'true', 'unsolved'),
    ([('function1', 'feasible')], 'task1 requirement', 'false', 'feasible'),
    ([('function1', 'unsolved')], 'task1 requirement', 'false', 'feasible'),
    ([('function1', 'configuration error'), ('function2', 'unsolved')], 'task1 requirement', 'false', 'feasible'),
    ([('function1', 'configuration error'), ('function2', 'implicit configuration error')], 'task1 requirement', 'false', 'feasible'),
    ([('function1', 'feasible'), ('function2', 'feasible')], 'task1 requirement', 'false', 'feasible'),
    ([('function1', 'solved'), ('function2', 'solved')], 'task1 requirement', 'true', 'solved'),
])
def test_task_requirement_status_inference(
     kb_interface, functions, tr_name, tr_required, tr_status):
    for f in functions:
        kb_interface.update_attribute_entity(
            'Function',
            'function-name',
            f[0],
            'function-status',
            "'{}'".format(f[1]))
    kb_interface.update_attribute_entity(
        'task-requirement',
        'task-requirement-name',
        tr_name,
        'is-required',
        tr_required)
    tr_status_inferred = kb_interface.get_attribute_from_entity(
        'task-requirement',
        'task-requirement-name',
        tr_name,
        'task-requirement-status')
    print(tr_status_inferred)
    assert all(x in tr_status_inferred for x in [tr_status]) is True


@pytest.mark.parametrize("tr_name, t_name, status", [
    ('task1 requirement', 'task1', 'unfeasible'),
    ('task1 requirement', 'task1', 'implicit configuration error'),
    ('task1 requirement', 'task1', 'unsolved'),
    ('task1 requirement', 'task1', 'feasible'),
    ('task1 requirement', 'task1', 'solved'),
])
def test_task_status_inference(kb_interface, tr_name, t_name, status):
    kb_interface.update_attribute_entity(
        'task-requirement',
        'task-requirement-name',
        tr_name,
        'task-requirement-status',
        "'{}'".format(status))
    tr_status_inferred = kb_interface.get_attribute_from_entity(
        'task-requirement',
        'task-requirement-name',
        tr_name,
        'task-requirement-status')
    t_status_inferred = kb_interface.get_attribute_from_entity(
        'Task',
        'task-name',
        t_name,
        'task-status')
    print(t_status_inferred)
    assert tr_status_inferred == t_status_inferred
