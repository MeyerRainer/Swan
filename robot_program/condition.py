from abc import ABC
from dataclasses import dataclass


class Condition(ABC):
    """ Base class for all boolean conditions.

    Conditions are parsed from the source program and later
    evaluated by the robot executor.
    """
    pass

@dataclass
class DigitalInputCondition(Condition):

    def __init__(self, signal: str):
        self.signal = signal

    signal: str