from abc import ABC, abstractmethod

class StateMachine(ABC):

    class State(ABC):

        @abstractmethod
        def get_enum(self):
            pass

        @abstractmethod
        def entry(self, machine):
            pass

        @abstractmethod
        def step(self, machine):
            pass

        @abstractmethod
        def transition(self, machine) -> str:
            pass

        def get_name(self):
            return self.__name

    def __init__(self, states, state):
        self.state = state
        self.states = states

        self.states[self.state].entry(self)

    def get_current_state(self) -> str:
        return self.state

    def step(self):
        initial_state = self.state
        self.states[self.state].step(self)
        self.state = self.states[self.state].transition(self)
        while self.state is not initial_state:
            self.states[self.state].entry(self)
            initial_state = self.state
            self.states[self.state].step(self)
            self.state = self.states[self.state].transition(self)

