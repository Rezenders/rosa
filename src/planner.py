

# TODO: Can be an abstract class
class Planner:
    def __init__(self, typedb_interface):
        self.typedb_interface = typedb_interface

    def plan(self):
        self.update_functional_requirement()
        self.update_component_requirement()
        self.define_adaptation_plan()

    # TODO: Can this be transformed into a rule?
    def update_functional_requirement(self):
        # TODO: fetch all required tasks? instead of just the unsolved ones?
        tasks = self.typedb_interface.get_unsolved_required_tasks()
        for task in tasks:
            functional_hierarchy = \
                self.typedb_interface.get_functional_hierarchy_from_task(task)

            for functions in functional_hierarchy.values():
                for function in functions:
                    self.typedb_interface.update_attribute_entity(
                        "Function",
                        "function-name",
                        function,
                        "is-function-required",
                        "true")
        self.remove_functional_requirement()

    def remove_functional_requirement(self):
        unrequired_functions = \
            self.typedb_interface.get_functions_not_required_anymore()
        for function in unrequired_functions:
            self.typedb_interface.update_attribute_entity(
                "Function",
                "function-name",
                function,
                "is-function-required",
                "false")

    # TODO: not sure if it is necessary to return a component_dict
    def update_component_requirement(self):

        required_functions = self.typedb_interface.get_required_functions()

        for function in required_functions:
            # TODO: return it unordered and order here?
            function_designs = \
                self.typedb_interface.get_function_designs_ordered(function)

            required_components = []  # is this used?
            # components_dict = dict()
            if len(function_designs) > 0:
                fd_name = function_designs[0]
                required_components_type = self.typedb_interface.get_component_types_from_function_design(fd_name)
                for component_type in required_components_type:
                    self.typedb_interface.update_attribute_entity(
                        "ComponentType",
                        "component-type",
                        component_type,
                        "is-component-type-required",
                        "true")
                    components_designs_ordered = self.typedb_interface.get_components_from_component_type(component_type)
                    if len(components_designs_ordered) > 0:
                        required_components += [components_designs_ordered[0]]

                required_components += self.typedb_interface.get_components_from_function_design(fd_name)
                print(required_components)
                for component in required_components:
                    self.typedb_interface.update_attribute_component(
                        component, 'is-component-required', "true")
                # components_dict.setdefault(function_name, []).append(component)
        self.remove_component_type_requirement()
        self.remove_component_requirement()
        # return components_dict

    def remove_component_type_requirement(self):
        unrequired_components_types = \
            self.typedb_interface.get_component_types_not_required_anymore()

        for component in unrequired_components_types:
            self.typedb_interface.update_attribute_entity(
                "ComponentType",
                "component-type",
                component,
                "is-component-type-required",
                "false")

    def remove_component_requirement(self):
        unrequired_components = \
            self.typedb_interface.get_components_not_required_anymore()

        for component in unrequired_components:
            self.typedb_interface.update_attribute_entity(
                "Component",
                "component-name",
                component,
                "is-component-required",
                "false")

    def define_adaptation_plan(self):
        pass
