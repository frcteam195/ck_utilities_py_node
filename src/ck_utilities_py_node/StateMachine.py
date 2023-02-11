from abc import ABC, abstractmethod

class StateMachine(ABC):

    class State(ABC):
        def __init__(self):
            pass

        @abstractmethod
        def get_name(self):

        @abstractmethod
        def entry(self):
            pass

        @abstractmethod
        def step(self):
            pass

        @abstractmethod
        def transition(self) -> str:
            pass

        def get_name(self):
            return self.__name

    def __init__(self):
        self.__state = ""
        self.__states = {}

        self.initialize_states()
        pass

    @abstractmethod
    def initialize_states(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    def get_current_state(self) -> str:
        if self.__state is not "":
            return self.__state
        return "None"

    def step(self):
        initial_state = self.__state
        self.__states[self.__state].step()
        self.__states[self.__state].transition()
        while self.__state is not initial_state:
            initial_state = self.__state
            self.__states[self.__state].step()
            self.__states[self.__state].transition()
