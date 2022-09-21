

# TODO: Can be an abstract class
class Planner:
    def __init__(self, typedb_interface):
        self.typedb_interface = typedb_interface

    def plan(self):
        tasks = self.typedb_interface.get_unsolved_required_tasks()
        for task in tasks:
            functional_hierarchy = \
                self.typedb_interface.get_functional_hierarchy_from_task(task)

            for functions in functional_hierarchy.values():
                for function in functions:
                    self.update_function_requirement(function)
                    self.select_required_components(function)

        self.remove_functions_requirement()

    # TODO: not sure if it is necessary to return a component_dict
    def select_required_components(self, function_name):
        # TODO: return it unordered and order here?
        function_designs = \
            self.typedb_interface.get_function_designs_ordered(function_name)

        required_components = [] # is this used?
        components_dict = dict()
        if len(function_designs) > 0:
            fd_name = function_designs[0]
            required_components_type = self.typedb_interface.get_component_types_from_function_design(fd_name)
            for component_type in required_components_type:
                components_designs_ordered = self.typedb_interface.get_components_from_component_type(component_type)
                if len(components_designs_ordered) > 0:
                    required_components += [components_designs_ordered[0]]

            required_components += self.typedb_interface.get_components_from_function_design(fd_name)

            for component in required_components:
                self.typedb_interface.update_attribute_component(
                    component, 'is-component-required', "true")
                components_dict.setdefault(function_name, []).append(component)
        return components_dict

    def update_function_requirement(self, function):
        self.typedb_interface.update_attribute_entity(
            "Function",
            "function-name",
            function,
            "is-function-required",
            "true")

    def remove_functions_requirement(self):
        unrequired_functions = \
            self.typedb_interface.get_functions_not_required_anymore()
        for function in unrequired_functions:
            self.typedb_interface.update_attribute_entity(
                "Function",
                "function-name",
                function,
                "is-function-required",
                "false")
