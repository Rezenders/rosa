from abc import ABC

# TODO: Can be an abstract class
class Analyze(ABC):
    @abstractmethod
    def analyze_fg_qas(self):
        pass

class DefaultAnalyze(Analyze):
    pass
