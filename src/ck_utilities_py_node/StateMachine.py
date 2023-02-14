from abc import ABC, abstractmethod
from queue import Queue
from enum import Enum
from ck_ros_base_msgs_node.msg import StateMachineLog
import rospy


class StateMachine(ABC):

    class State(ABC):

        @abstractmethod
        def get_enum(self):
            pass

        @abstractmethod
        def entry(self):
            pass

        @abstractmethod
        def step(self):
            pass

        @abstractmethod
        def transition(self) -> Enum:
            pass

        def get_name(self):
            return self.__name

    def __init__(self, states, state):
        self.state = state
        self.states = states

        self.states[self.state].entry()
        self.transition_history = Queue(10)

        self.log_count = 0
        self.transition_history.put(str(self.log_count) + ": Init: " + str(self.state))
        self.log_count += 1
        self.log_publisher = rospy.Publisher(name="/state_machines/" + str(self.__class__.__name__),
                                             data_class=StateMachineLog,
                                             queue_size=100,
                                             tcp_nodelay=False)

    def get_current_state(self) -> str:
        return self.state

    def log_data(self):
        log_string = ""
        for i in self.transition_history.queue:
            log_string += i + "\n"
        log_msg = StateMachineLog()
        log_msg.current_state = str(self.state)
        log_msg.current_state_int = self.state.value
        log_msg.transition_history = log_string
        self.log_publisher.publish(log_msg)

    def step(self):
        initial_state = self.state
        self.states[self.state].step()
        self.state = self.states[self.state].transition()
        while self.state is not initial_state:
            if self.transition_history.full():
                self.transition_history.get()
            self.transition_history.put(str(self.log_count) + ": TRANSITION: " + str(initial_state) + " -> " + str(self.state))
            self.log_count += 1
            self.states[self.state].entry()
            initial_state = self.state
            self.states[self.state].step()
            self.state = self.states[self.state].transition()
        # self.log_data()
