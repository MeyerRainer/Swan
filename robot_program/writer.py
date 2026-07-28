from abc import ABC, abstractmethod

from robot_program.program import Program


class ProgramWriter(ABC):

    @abstractmethod
    def write(self,
              filename: str,
              program: Program):

        raise NotImplementedError