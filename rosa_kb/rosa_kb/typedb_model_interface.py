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
"""python interface to interact with typedb's ROSA knowledge model."""

from ros_typedb.typedb_interface import TypeDBInterface
from datetime import datetime

from typedb.driver import ConceptMap

from typing import Iterator
from typing import Literal
from typing import Tuple
from typing import TypedDict
from typing import Optional


class MatchResultDict(TypedDict):
    """TypedDict for match result."""

    type: str  #: attribute name, e.g., name, age, height etc
    value_type: str  #: value type, e.g., boolean, long etc
    value: str  #: value


class ReconfigPlanDict(TypedDict):
    """TypedDict for reconfiguration plan."""

    start_time: datetime  #: reconfig plan start-time
    c_activate: list[str]  #: components to activate
    c_deactivate: list[str]  #: components to deactivate
    c_config: list[str]  #: component configurations to update


class ComponentParemeterDict(TypedDict):
    """TypedDict for ComponentParemeter."""

    key: str  #: component parameter key
    value: bool | list[bool] | float | list[float] | int | list[int] | str | \
        list[str]  #: component parameter value
    type: Literal[
        'boolean',
        'boolean_array',
        'double',
        'double_array',
        'long',
        'long_array',
        'string',
        'string_array']  #: component parameter type


class ComponentConfigurationDict(TypedDict):
    """TypedDict for component-configuration."""

    component: str  #: component name
    component_parameter: list[ComponentParemeterDict]  #: list with parameters


class ComponentDict(TypedDict):
    """TypedDict for Component."""

    type: str  #: the typedb type of the component
    component_name: str  #: component name
    is_required: bool  #: whether component is required or not
    is_active: bool  #: whether component is active or not
    component_status: str  #: component status
    package: str  #: package that contains the component
    executable: str  #: executable that starts the component


def convert_component_parameter_value_to_py_type(
    param: dict[str, MatchResultDict],
    param_type: Literal[
        'boolean', 'boolean_array', 'double', 'double_array',
        'long', 'long_array', 'string', 'string_array']
     ) -> bool | list[bool] | float | list[float] | int | list[int] | str | \
      list[str]:
    """
    Convert ComponentParameter value to python type.

    :param param: ComponentParameter value
    :param param_type: CompomentParameter parameter-type
    :return: converted value
    """
    def process_array(param, func):
        return [func(p.strip()) for p in param.strip('[]').split(',')]

    if param_type == 'boolean':
        return param.lower() == 'true'
    elif param_type == 'boolean_array':
        return process_array(param, lambda p: p.lower() == 'true')
    elif param_type == 'double':
        return float(param)
    elif param_type == 'double_array':
        return process_array(param, float)
    elif param_type == 'long':
        return int(param)
    elif param_type == 'long_array':
        return process_array(param, int)
    elif param_type == 'string':
        return param
    elif param_type == 'string_array':
        return process_array(param, str)


class ModelInterface(TypeDBInterface):
    """Class to interact with the ROSA knowledge model in typeDB."""

    def __init__(
            self,
            address: str,
            database_name: str,
            schema_path: Optional[list[str] | str] = None,
            data_path: Optional[list[str] | str] = None,
            force_database: Optional[bool] = False,
            force_data: Optional[bool] = False,
            infer: Optional[bool] = False) -> None:

        super().__init__(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data,
            infer
        )

    def request_action(self, action_name: str) -> Iterator[ConceptMap] | None:
        """
        Request Action.

        :param action_name: action name
        :return: insert query result
        """
        return self.update_attribute_in_thing(
            'Action', 'action-name', action_name, 'is-required', True)

    def cancel_action(self, action_name: str) -> Iterator[ConceptMap] | None:
        """
        Cancel Action.

        :param action_name: action name
        :return: insert query result
        """
        return self.update_attribute_in_thing(
            'Action', 'action-name', action_name, 'is-required', False)

    def update_action_status(
            self,
            action_name: str,
            action_status: str) -> Iterator[ConceptMap] | None:
        """
        Update Action status.

        :param action_name: action name
        :param action_status: action_status
        :return: insert query result
        """
        return self.update_attribute_in_thing(
            'Action',
            'action-name',
            action_name,
            'action-status',
            action_status
        )

    def delete_component_status(
            self, component_name: str) -> Literal[True] | None:
        """
        Delete Component status.

        :param component_name: component name
        :return: delete query result
        """
        return self.delete_attribute_from_thing(
            'Component',
            'component-name',
            component_name,
            'component-status')

    def update_component_status(
            self,
            component_name: str,
            component_status: str) -> Iterator[ConceptMap] | None:
        """
        Update Component status.

        :param component_name: component name
        :return: delete query result
        """
        return self.update_attribute_in_thing(
            'Component',
            'component-name',
            component_name,
            'component-status',
            component_status
        )

    def is_action_required(self, action_name: str) -> bool:
        """
        Check whether an Action is required.

        :param action_name: Action name
        :return: whether the action is required or not
        """
        is_required = self.get_attribute_from_thing(
             'Action', [('action-name', action_name)], 'is-required')
        if len(is_required) == 0:
            return False
        return is_required[0]

    def is_action_feasible(self, action_name: str) -> bool:
        """
        Check if an Action only has status feasible.

        :param action_name: Action name
        :return: whether the action is feasible or not
        """
        status = self.get_attribute_from_thing(
             'Action', [('action-name', action_name)], 'action-status')
        return all(x in status for x in ['feasible'])

    def is_action_selectable(self, action_name: str) -> bool:
        """
        Check whether an Action can be selected, i.e., it can be performed.

        :param action_name: Action name
        :return: whether the action is feasible or not
        """
        status = self.get_attribute_from_thing(
             'Action', [('action-name', action_name)], 'action-status')
        return 'unfeasible' not in status

    def get_selectable_thing_raw(
         self, thing: str) -> list[dict[str, MatchResultDict]]:
        """
        Get the name of selectable individuals of a specific Thing.

        :param thing: Thing type
        :return: name of selectable individuals
        """
        query = f'''
            match
            $t isa {thing}, has name $name;
            not {{$t has status 'unfeasible';}};
            get $name;
        '''
        query_result = self.match_database(query)
        return query_result

    def get_selectable_actions(self) -> list[str]:
        """
        Get the name of selectable Actions.

        :return: name of selectable actions
        """
        query_result = self.get_selectable_thing_raw('Action')
        return [r.get('name').get('value') for r in query_result]

    def get_instances_of_thing_with_status(
            self, thing: str, status: str) -> list[str]:
        """
        Get name of instances of a certain Thing that have a certain status.

        :param thing: thing type to query.
        :param status: status.
        :return: name of instances with status.
        """
        query = f'''
            match
                $e isa {thing},
                    has {thing.lower()}-name $name,
                    has {thing.lower()}-status $status;
                    $status like "{status}";
                get $name;
        '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def get_solved_functions(self) -> list[str]:
        """Get functions with status solved."""
        return self.get_instances_of_thing_with_status(
            'Function', 'solved')

    def get_solved_components(self) -> list[str]:
        """Get components with status solved."""
        return self.get_instances_of_thing_with_status(
            'Component', 'solved')

    def get_instances_thing_always_improve(self, thing: str) -> list[str]:
        """
        Get name of instances of a certain Thing that have always-improve true.

        :param thing: Thing type to query.
        :return: name of instances with always-improve true.
        """
        query = f'''
            match
                $f isa {thing}, has always-improve true,
                    has {thing.lower()}-name $name;
                get $name;
        '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def get_adaptable_things_raw(
         self, thing: str) -> list[dict[str, MatchResultDict]]:
        """
        Get the name of adaptable individuals of a certain Thing.

        Get the name of adaptable individuals of a certain Thing. An individual
        is adaptable when it has an 'always-improve' attribute set to true, or
        when it has status 'unsolved' or 'configuration error'

        :param thing: Thing type
        :return: name of adaptable individuals
        """
        query = f'''
            match
                $t isa {thing}, has {thing.lower()}-name $name;
                {{
                    $t has {thing.lower()}-status $status;
                        $status like 'unsolved|configuration error';
                }} or {{
                    $t has always-improve true;
                    $t has {thing.lower()}-status $status;
                        $status like 'solved';
                }};
                get $name;
        '''
        query_result = self.match_database(query)
        return query_result

    def get_adaptable_functions(self) -> list[str]:
        """
        Get the name of adaptable Functions.

        Get the name of adaptable Functions. A function is adaptable when its
        'always-improve' attribute is true, or when it status is 'unsolved' or
        'configuration error'

        :return: name of adaptable functions
        """
        query_result = self.get_adaptable_things_raw('Function')
        return [r.get('name').get('value') for r in query_result]

    def get_adaptable_components(self) -> list[str]:
        """
        Get the name of adaptable Components.

        Get the name of adaptable Components. A component is adaptable when its
        'always-improve' attribute is true, or when it status is 'unsolved' or
        'configuration error'

        :return: name of adaptable components
        """
        query_result = self.get_adaptable_things_raw('Component')
        return [r.get('name').get('value') for r in query_result]

    def get_unsolved_thing_raw(
         self, thing: str) -> list[dict[str, MatchResultDict]]:
        """
        Get unsolved individuals of a certain Thing type.

        Get unsolved individuals of a certain Thing type. An individual is
        unsolved when its 'is-required' attribute is True and its status is
        'unsolved'.

        :return: name of unsolved individuals
        """
        query = f'''
            match
                $e isa {thing}, has is-required true,
                    has {thing.lower()}-name $name,
                    has {thing.lower()}-status 'unsolved';
                get $name;
        '''
        return self.match_database(query)

    def get_unsolved_functions(self) -> list[str]:
        """
        Get unsolved Functions.

        Get unsolved Functions. A function is unsolved when its 'is-required'
        attribute is True and its status is 'unsolved'.

        :return: name of unsolved functions
        """
        query_result = self.get_unsolved_thing_raw('Function')
        return [r.get('name').get('value') for r in query_result]

    def get_unsolved_components(self) -> list[str]:
        """
        Get unsolved Components.

        Get unsolved Components. A component is unsolved when its 'is-required'
        attribute is True and its status is 'unsolved'.

        :return: name of unsolved components
        """
        query_result = self.get_unsolved_thing_raw('Component')
        return [r.get("name").get('value') for r in query_result]

    def update_function_design_priority(
            self, fd_name: str, value: float) -> Iterator[ConceptMap] | None:
        """
        Update function design priority.

        :param fd_name: function design name
        :param value: new priority value
        :return: query result
        """
        return self.update_attribute_in_thing(
            'function-design',
            'function-design-name',
            fd_name,
            'priority',
            value)

    def add_measurement(
         self, name: str, value: str) -> Iterator[ConceptMap] | None:
        """
        Add new Quality Attribute or EnvironmentalAttribute measurement.

        Add new Quality Attribute or EnvironmentalAttribute measurement. The
        new measurement 'latest' attribute is set to true, and the old
        measurements 'latest' attribute are set to false.

        :param name: QA/EA name
        :param value: measured value
        :return: query result
        """
        query = f"""
            match
                $attr isa Attribute, has attribute-name "{name}";
                $m (measured-attribute:$attr) isa measurement,
                    has latest $latest;
                $latest == true;
            delete $m has $latest;
        """
        self.delete_from_database(query)

        time = self.convert_py_type_to_query_type(datetime.now())
        query = f"""
            match
                $attr isa Attribute, has attribute-name "{name}";
            insert
                $m (measured-attribute:$attr) isa measurement,
                    has latest true,
                    has measurement-value {value},
                    has measurement-time {time};
        """
        return self.insert_database(query)

    def get_latest_measurement(self, name: str) -> float | None:
        """
        Get latest measurement value.

        :param name: QA/EA name
        :return: measurement value, or None if there is no measurement
        """
        query = f"""
            match
                $attr isa Attribute, has attribute-name "{name}";
                $m (measured-attribute:$attr) isa measurement,
                    has latest true , has measurement-value $value;
                get $value;
        """
        query_result = self.match_database(query)
        if len(query_result) == 0:
            return None
        return query_result[0].get('value').get('value')

    def get_measurement(self, name: str, time: datetime) -> float | None:
        """
        Get measured value at a certain time.

        :param name: QA/EA name
        :param time: measurement time
        :return: measurement value, or None if there is no measurement
        """
        time = self.convert_py_type_to_query_type(time)
        query = f"""
            match
                $attr isa Attribute,
                    has attribute-name "{name}";
                $m (measured-attribute:$attr) isa measurement,
                    has measurement-time {time} , has measurement-value $value;
                get $value;
        """
        query_result = self.match_database(query)
        if len(query_result) == 0:
            return None
        return query_result[0].get('value').get('value')

    def get_selectable_c_configs(self, component_name: str) -> list[str]:
        """
        Get the name of selectable component configurations for a Component.

        :param component_name: component name
        :return: name of selectable component configurations
        """
        query = f'''
            match
                $c isa Component, has component-name "{component_name}";
                $cc (component: $c) isa component-configuration,
                    has component-configuration-name $name;
                not {{
                    $cc has component-configuration-status 'unfeasible';
                }};
                get $name;
        '''
        result = self.match_database(query)
        return [r.get('name').get('value') for r in result]

    def get_selectable_fds(self, function_name: str) -> list[str]:
        """
        Get the name of selectable funtion designs for a Function.

        :param function_name: function name
        :return: name of selectable function designs
        """
        query = f'''
            match
                $f isa Function, has function-name "{function_name}";
                $fd (function: $f) isa function-design,
                    has function-design-name $name;
                not {{
                    $fd has function-design-status 'unfeasible';
                }};
                get $name;
        '''
        result = self.match_database(query)
        return [r.get('name').get('value') for r in result]

    def get_function_design_priority(self, fd_name: str) -> list[str]:
        """
        Get function design priority value.

        :param fd_name: function design name
        :return: function design priority value
        """
        return self.get_attribute_from_thing(
            'function-design', [('function-design-name', fd_name)], 'priority')

    def get_component_configuration_priority(self, cc_name: str) -> list[str]:
        """
        Get component configuration priority value.

        :param cc_name: component configuration name
        :return: component configuration priority value
        """
        return self.get_attribute_from_thing(
            'component-configuration',
            [('component-configuration-name', cc_name)],
            'priority')

    def get_relationship_with_attribute(
            self,
            entity: str,
            entity_name: str,
            relation: str,
            r_attribute: str,
            r_value: str | int | float | bool | datetime) -> list[str]:
        """
        Get relationship name that has an attr and relates to a certain entity.

        Get relationship name that has an attr and relates to a certain entity.
        For example, a 'function-design' that has the 'is-selected' attribute
        set to true and is related to a 'Function' with 'function-name'
        'my_function'.

        :param entity: entity type
        :param entity_name: entity name
        :param relation: relation type
        :param r_attribute: attribute name
        :param r_value: attribute value
        :return: name of individuals of the relation
        """
        entity_name = self.convert_py_type_to_query_type(entity_name)
        r_value = self.convert_py_type_to_query_type(r_value)
        query = f'''
            match
                $e isa {entity},
                    has {entity.lower()}-name {entity_name};
                $r ($e) isa {relation},
                    has {r_attribute} {r_value},
                    has {relation}-name $name;
                get $name;
            '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def select_relationship(
            self,
            entity: str,
            entity_name: str,
            relation: str,
            r_name: str) -> Iterator[ConceptMap] | None:
        """
        Select relationship individual, and unselect all other individuals.

        Set the 'is-selected' attribute of the 'r_name' individual to true, and
        set 'is-selected' to false for all other 'relation' individuals related
        to 'entity_name'.

        :param entity: entity type
        :param entity_name: entity name
        :param relation: relation type
        :param r_name: relationship name
        :return: update query result
        """
        current_selected = self.get_relationship_with_attribute(
            entity, entity_name, relation, 'is-selected', True)
        for r in current_selected:
            if r != r_name:
                self.update_attribute_in_thing(
                    relation,
                    '{}-name'.format(relation),
                    r,
                    'is-selected',
                    False)
        return self.update_attribute_in_thing(
            relation,
            '{}-name'.format(relation),
            r_name,
            'is-selected',
            True)

    def select_function_design(
            self, f_name: str, fd_name: str) -> Iterator[ConceptMap] | None:
        """
        Select function-design 'fd_name' and unselect all other fds.

        Set the 'is-selected' attribute of the 'fd_name' function design to
        true, and set 'is-selected' to false for all other function designs
        related to 'f_name'.

        :param f_name: function name
        :param fd_name: function design name
        :return: update query result
        """
        return self.select_relationship(
            'Function', f_name, 'function-design', fd_name)

    def select_component_configuration(
            self, c_name: str, cc_name: str) -> Iterator[ConceptMap] | None:
        """
        Select component-configuration 'cc_name' and unselect all other ccs.

        Set the 'is-selected' attribute of the 'cc_name' component
        configuration to true, and set 'is-selected' to false for all other
        component configurations related to 'c_name'.

        :param c_name: component name
        :param cc_name: component configuration name
        :return: update query result
        """
        return self.select_relationship(
            'Component', c_name, 'component-configuration', cc_name)

    def activate_component(
            self, c_name: str, value: bool) -> Iterator[ConceptMap] | None:
        """
        Activate a Component.

        Set 'is-active' attribute of a Component to true or false.

        :param c_name: component name
        :param value: whether the component should be active or not
        :return: update query result
        """
        return self.update_attribute_in_thing(
            'Component',
            'component-name',
            c_name,
            'is-active',
            value)

    def is_component_active(self, name: str) -> bool:
        """
        Check wheter a component is active or not.

        :param name: component name
        :return: whether the component is active or not
        """
        is_activated = self.get_attribute_from_thing(
            'Component',
            [('component-name', name)],
            'is-active')
        if len(is_activated) == 0:
            return None
        return is_activated[0]

    def create_reconfiguration_plan(
            self,
            c_activate: list[str],
            c_deactivate: list[str],
            c_config: list[str]
         ) -> datetime | None:
        """
        Create a reconfiguration plan.

        :param c_active: components to activate
        :param c_deactive: components to deactivate
        :param c_config: component configurations to select
        :return: the time the reconfiguration plan was created, or None in case
            there was a failure creating the reconfiguration plan
        """
        match_query = "match "
        insert_query = "insert "

        if len(c_activate) == 0 and len(c_deactivate) == 0 and \
           len(c_config) == 0:
            return None

        structural_adaptation = []
        if len(c_activate) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('Component', 'component-name', c) for c in c_activate], 'ca')
            match_query += _match_query

            insert_query += self.create_relationship_query(
                'component-activation',
                {'component': _prefix_list},
                prefix='rca'
            )
            structural_adaptation.append('rca')

        if len(c_deactivate) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('Component', 'component-name', c) for c in c_deactivate],
                'cd')
            match_query += _match_query

            insert_query += self.create_relationship_query(
                'component-deactivation',
                {'component': _prefix_list},
                prefix='rcd'
            )
            structural_adaptation.append('rcd')

        parameter_adaptation = []
        if len(c_config) > 0:
            _match_query, _prefix_list = self.create_match_query(
                [('component-configuration', 'component-configuration-name', c)
                    for c in c_config],
                'cc'
            )
            match_query += _match_query

            insert_query += self.create_relationship_query(
                'parameter-adaptation',
                {'component-configuration': _prefix_list},
                prefix='rcc'
            )
            parameter_adaptation.append('rcc')

        start_time = datetime.now()
        insert_query += self.create_relationship_query(
            'reconfiguration-plan',
            {
                'structural-adaptation': structural_adaptation,
                'parameter-adaptation': parameter_adaptation
            },
            attribute_list=[('start-time',  start_time)],
            prefix='rp'
        )

        query = match_query + insert_query
        insert_result = self.insert_database(query)
        if insert_result is None:
            return None
        return datetime.fromisoformat(
            start_time.isoformat(timespec='milliseconds'))

    def select_fd_and_get_components(
            self,
            functions_selected_fd: list[Tuple[str, str]]
         ) -> Tuple[list[str], list[str]]:
        """
        Select function design, and get components to activate and deactivate.

        Set the 'is-selected' attribute of the function desings in
        `functions_selected_fd` to true, and returns a tuple with a list with
        the components that need to be activated and a list with the components
        that need to be deactivated.

        :param functions_selected_fd: a list of tuples with the form
            (FUNCTION_NAME, FD_NAME) representing the function designs that
            were selected for which function
        :return: tuple with the form (c_activate, c_deactivate), indicating
            which components should be activated and deactivated
        """
        _c_activate = []
        _c_deactivate = []
        for function, fd in functions_selected_fd:
            # select components that need to be activated
            for c in self.get_components_in_function_design(fd):
                c_active = self.get_attribute_from_thing(
                    'Component', [('component-name', c)], 'is-active')
                if True not in c_active:
                    _c_activate.append(c)
            # select components that need to be deactivated
            fd_selected = self.get_attribute_from_thing(
                'function-design',
                [('function-design-name', fd)],
                'is-selected')
            if len(fd_selected) == 0  \
               or len(fd_selected) > 0 and fd_selected[0] is False:
                _fd = self.get_relationship_with_attribute(
                    'Function',
                    function,
                    'function-design',
                    'is-selected',
                    True
                )
                if len(_fd) > 0:
                    for c in self.get_components_in_function_design(_fd[0]):
                        c_active = self.get_attribute_from_thing(
                            'Component', [('component-name', c)], 'is-active')
                        if c not in _c_activate and len(c_active) > 0 \
                           and c_active[0] is True:
                            _c_deactivate.append(c)
            self.select_function_design(function, fd)
        return _c_activate, _c_deactivate

    def select_components_selected_config(
            self,
            components_selected_config: list[Tuple[str, str]]
         ) -> list[str]:
        """
        Select component configuration and return them.

        Set the 'is-selected' attribute of the component configurations in
        `components_selected_config` to true, and returns a list with
        the name of the component configurations that need to be updated.

        :param components_selected_config: a list of tuples with the form
            (COMPONENT_NAME, CC_NAME) representing the component configurations
            that were selected for which components
        :return: list with the name of the component configurations that need
            to be updated
        """
        _configs = []
        for component, config in components_selected_config:
            config_selected = self.get_attribute_from_thing(
                'component-configuration',
                [('component-configuration-name', config)],
                'is-selected'
            )
            if len(config_selected) == 0  \
               or len(config_selected) > 0 and config_selected[0] is False:
                _configs.append(config)
            self.select_component_configuration(component, config)
        return _configs

    def get_obsolete_components(self) -> list[str]:
        """
        Get active components that are not required anymore.

        :return: List with active components that are not required anymore
        """
        query = '''
            match
            $c isa Component, has component-name $name, has is-active true;
            not {
                $fd (function:$func, required-component:$c)
                    isa function-design;
                $func has is-required true;
            };
            get $name;
        '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def get_obsolete_fds(self) -> list[str]:
        """
        Get selected fds that are not required anymore.

        :return: List with selected fds that are not required anymore
        """
        query = '''
            match
                $fd (function:$f) isa function-design,
                    has function-design-name $name, has is-selected true;
                not {$f has is-required true;};
            get $name;
        '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def get_obsolete_component_configurations(self) -> list[str]:
        """
        Get selected component configurations that are not required anymore.

        :return: List with selected configs that are not required anymore
        """
        query = '''
            match
                $cc (component:$c) isa component-configuration,
                    has component-configuration-name $name,
                    has is-selected true;
                not {
                    $fd (function:$func, required-component:$c)
                        isa function-design;
                    $func has is-required true;
                };
        '''
        query_result = self.match_database(query)
        return [r.get('name').get('value') for r in query_result]

    def unselect_obsolete_fds_cc(self) -> None:
        """
        Unselect all obsolete function designs and component configurations.

        An obsolete function design or component configuration is the one that
        has 'is-selected' attribute set to true, but it is no required anymore
        """
        _fds = self.get_obsolete_fds()
        for _fd in _fds:
            self.update_attribute_in_thing(
                'function-design',
                'function-design-name',
                _fd,
                'is-selected',
                False)

        _ccs = self.get_obsolete_component_configurations()
        for _cc in _ccs:
            self.update_attribute_in_thing(
                'component-configuration',
                'component-configuration-name',
                _cc,
                'is-selected',
                False)

    def select_configuration(
         self,
         functions_selected_fd: list[Tuple[str, str]],
         components_selected_config: list[Tuple[str, str]]) -> datetime | None:
        """
        Select configuration and create reconfiguration plan.

        :param functions_selected_fd: a list of tuples with the form
            (FUNCTION_NAME, FD_NAME) representing the function designs that
            were selected for which function
        :param components_selected_config:a list of tuples with the form
            (COMPONENT_NAME, CC_NAME) representing the component configurations
            that were selected for which components
        :return: reconfig plan creation time.
        """
        _c_activate, _c_deactivate = self.select_fd_and_get_components(
            functions_selected_fd)
        _configs = self.select_components_selected_config(
            components_selected_config)

        self.unselect_obsolete_fds_cc()
        _c_obsolete = self.get_obsolete_components()
        _c_deactivate.extend(
            c for c in _c_obsolete if
            (c not in _c_activate and c not in _c_deactivate))

        return self.create_reconfiguration_plan(
            _c_activate, _c_deactivate, _configs)

    def get_components_in_function_design(self, fd_name: str) -> list[str]:
        """
        Get components in relation with a function design.

        :param fd_name: name of the function design.
        :return: component names in relation with fd_name.
        """
        query = f'''
            match
                $fd (required-component: $component) isa function-design,
                    has function-design-name "{fd_name}";
                $component isa Component, has component-name $c-name;
                get $c-name;
            '''
        query_result = self.match_database(query)
        return [r.get('c-name').get('value') for r in query_result]

    def get_latest_reconfiguration_plan_time(self) -> datetime | None:
        """
        Get start-time of the most recent reconfiguration plan.

        :return: start-time of the most recent reconfiguration plan.
        """
        query = '''
            match $rp isa reconfiguration-plan, has start-time $time;
            get $time;
            sort $time desc; limit 1;
        '''
        result = self.match_database(query)
        if len(result) == 0:
            return None
        return self.convert_query_type_to_py_type(result[0].get('time'))

    def get_latest_pending_reconfiguration_plan_time(self) -> datetime | None:
        """
        Get start-time of the most recent pending reconfiguration plan.

        :return: start-time of the most recent pending reconfiguration plan.
        """
        query = '''
            match
            not {$rp isa reconfiguration-plan, has result $result;};
            $rp has start-time $time;
            sort $time desc; limit 1;
        '''
        result = self.match_database(query)
        if len(result) == 0:
            return None
        return self.convert_query_type_to_py_type(result[0].get('time'))

    def get_latest_completed_reconfiguration_plan_time(
            self) -> datetime | None:
        """
        Get end-time of the most recent completed reconfiguration plan.

        :return: end-time of the most recent completed reconfiguration plan.
        """
        query = '''
            match $rp isa reconfiguration-plan,
                has end-time $time,
                has result 'completed';
            get $time;
            sort $time desc; limit 1;
        '''
        result = self.match_database(query)
        if len(result) == 0:
            return None
        return self.convert_query_type_to_py_type(result[0].get('time'))

    def get_reconfiguration_plan(
         self, start_time: datetime) -> ReconfigPlanDict:
        """
        Get reconfiguration plan with start-time.

        :param start_time: start-time of the desired reconfiguration plan.
        :return: dict representing the reconfiguration plan, its keys are:
            start_time, c_activate, c_deactivate, c_config
        """
        query = f'''
            match (structural-adaptation:$ca_) isa reconfiguration-plan,
                has start-time {start_time.isoformat(timespec='milliseconds')};
            $ca_ (component:$ca) isa component-activation;
            $ca isa Component, has component-name $c_activate;
            get $c_activate;
        '''
        result = self.match_database(query)
        c_activate = [r.get('c_activate').get('value') for r in result]

        query = f'''
            match (structural-adaptation:$cd_) isa reconfiguration-plan,
                has start-time {start_time.isoformat(timespec='milliseconds')};
            $cd_ (component:$cd) isa component-deactivation;
            $cd isa Component, has component-name $c_deactivate;
            get $c_deactivate;
        '''
        result = self.match_database(query)
        c_deactivate = [r.get('c_deactivate').get('value') for r in result]

        query = f'''
            match (parameter-adaptation:$pa) isa reconfiguration-plan,
                has start-time {start_time.isoformat(timespec='milliseconds')};
            $pa (component-configuration:$cc) isa parameter-adaptation;
            $cc isa component-configuration,
                has component-configuration-name $c_config;
            get $c_config;
        '''
        result = self.match_database(query)
        c_config = [r.get('c_config').get('value') for r in result]
        reconfig_plan_dict = {
            'start_time': start_time,
            'c_activate': c_activate,
            'c_deactivate': c_deactivate,
            'c_config': c_config,
        }
        return reconfig_plan_dict

    def get_latest_reconfiguration_plan(self) -> ReconfigPlanDict | None:
        """
        Get latest reconfiguration plan.

        :return: dict representing the reconfiguration plan, its keys are:
            start_time, c_activate, c_deactivate, c_config
        """
        time = self.get_latest_reconfiguration_plan_time()
        if time is None:
            return None
        reconfig_plan_dict = self.get_reconfiguration_plan(time)
        reconfig_plan_dict['start_time'] = time
        return reconfig_plan_dict

    def get_latest_pending_reconfiguration_plan(
         self) -> ReconfigPlanDict | None:
        """
        Get latest pending reconfiguration plan.

        :return: dict representing the reconfiguration plan, its keys are:
            start_time, c_activate, c_deactivate, c_config
        """
        time = self.get_latest_pending_reconfiguration_plan_time()
        if time is None:
            return None
        reconfig_plan_dict = self.get_reconfiguration_plan(time)
        reconfig_plan_dict['start_time'] = time
        return reconfig_plan_dict

    def update_reconfiguration_plan_result(
         self,
         start_time: str | datetime,
         result_value: Literal['completed', 'failed', 'abandoned']
         ) -> Iterator[ConceptMap] | None:
        """
        Update reconfiguration plan result.

        :param start_time: reconfiguration plan start_time
        :param result_value: reconfig plan result
        :return: update query result
        """
        if type(start_time) is str:
            start_time = datetime.fromisoformat(start_time)
        match_dict = {
            'reconfiguration-plan': [
                {
                    'attributes': {
                        'start-time': start_time
                    },
                    'update_attributes': {
                        'end-time': datetime.now(),
                        'result': result_value,
                    }
                }
            ]
        }
        return self.update_attributes_in_thing(match_dict)

    def get_outdated_reconfiguration_plans(self) -> list[datetime]:
        """
        Get outdated reconfiguration plans.

        A reconfiguration plan is considered outdated when it doesn't have an
        `end-time` and its `start-time` is lower than the `end-time` of a
        reconfiguration plan with result 'completed'.

        :return: start-time of all outdated reconfiguration plans
        """
        end_time = self.get_latest_completed_reconfiguration_plan_time()
        if type(end_time) is datetime:
            end_time = self.convert_py_type_to_query_type(end_time)
            query = f'''
                match $rp isa reconfiguration-plan, has start-time $time;
                    not {{$rp has end-time $end-time;}};
                    $time < {end_time};
                get $time;
            '''
            result = self.match_database(query)
            if len(result) > 0:
                return [self.convert_query_type_to_py_type(r.get('time'))
                        for r in result]
        return []

    def update_outdated_reconfiguration_plans_result(
         self) -> Iterator[ConceptMap] | None:
        """
        Set outdated reconfiguration plans result to 'abandoned'.

        :return: update query result
        """
        outdated_times = self.get_outdated_reconfiguration_plans()
        if len(outdated_times) == 0:
            return None

        update_plans = [{
            'attributes': {'start-time': time},
            'update_attributes': {
                'end-time': datetime.now(),
                'result': 'abandoned'}
            } for time in outdated_times]

        match_dict = {
            'reconfiguration-plan': update_plans
        }
        return self.update_attributes_in_thing(match_dict)

    def get_reconfiguration_plan_result(
            self, start_time: datetime) -> str | None:
        """
        Get result of recongiration plan with `start_time`.

        :param start_time: reconfiguration plan start-time
        :return: reconfiguration plan result
        """
        start_time = self.convert_py_type_to_query_type(start_time)
        query = f'''
            match $rp isa reconfiguration-plan,
                has start-time {start_time},
                has result $result;
            get $result;
        '''
        result = self.match_database(query)
        if len(result) == 0:
            return None
        return self.convert_query_type_to_py_type(result[0].get('result'))

    def get_component_parameters(
            self, c_config: str) -> ComponentConfigurationDict | None:
        """
        Get ComponentParameters in a component configuration relationship.

        :param c_config: component configuration name.
        :return: Dict with component-name, parameter-key, parameter-value
        """
        query = f'''
            match
                (component:$c, parameter:$p) isa component-configuration,
                has component-configuration-name '{c_config}';
                $c has component-name $c_name;
                $p has parameter-key $key,
                    has parameter-value $value,
                    has parameter-type $type;
            get $c_name, $key, $value, $type;
        '''
        _result = self.match_database(query)
        if len(_result) == 0:
            return None

        params = []
        result = {}
        result['component'] = self.convert_query_type_to_py_type(
            _result[0].get('c_name'))
        for r in _result:
            value = convert_component_parameter_value_to_py_type(
                self.convert_query_type_to_py_type(r.get('value')),
                self.convert_query_type_to_py_type(r.get('type'))
            )
            params.append({
                'key': self.convert_query_type_to_py_type(r.get('key')),
                'value': value,
                'type': self.convert_query_type_to_py_type(r.get('type'))
            })
        result['component_parameters'] = params
        return result

    def get_component_all_attributes(
         self, component: str) -> ComponentDict | None:
        """
        Get all attributes owned by a Component, and the Component type.

        :param component: component name.
        :return: Dict with component type and all its attributes
        """
        query = f'''
            match
                $c isa! $component-type,
                    has component-name '{component}',
                    has $attribute;
                get $component-type, $attribute;
        '''
        result = self.match_database(query)
        if len(result) == 0:
            return None

        result_dict = {}
        result_dict['type'] = result[0].get('component-type').get('label')
        for r in result:
            attr_name = r.get('attribute').get('type').replace('-', '_')
            attr_value = self.convert_query_type_to_py_type(
                r.get('attribute'))
            result_dict[attr_name] = attr_value
        return result_dict
