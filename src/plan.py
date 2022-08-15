from abc import ABC

# TODO: Can be an abstract class
class Plan:
    @abstractmethod
    def solve_objectives(self):
        pass

    @abstractmethod
    def request_reconfiguration(self):
        pass
