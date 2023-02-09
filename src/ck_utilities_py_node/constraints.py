from typing import List, Callable
import rospy

class Constraint:
    def __init__(self, constraint_predicate : Callable[[], bool], limit_value_if_constrained : float) -> None:
        self.__constraint_predicate = constraint_predicate
        self.__limit_value_if_constrained = limit_value_if_constrained

    def is_constrained(self) -> bool:
        return self.__constraint_predicate()

    def get_limit_value(self) -> float:
        return self.__limit_value_if_constrained

class ConstrainedComponent:
    def __init__(self, constraint_list : List[Constraint]):
        self.__constraint_list = constraint_list

    def get_output_value(self, original_value : float) -> float:
        limit_value_list : List[float] = []
        for c in self.__constraint_list:
            if c.is_constrained():
                limit_value_list.append(c.get_limit_value())

        list_length = len(limit_value_list)
        if list_length > 0:
            if list_length > 1:
                rospy.logerr("Constraints currently have more than one active constraint. Please adjust constraints or implement a resolution strategy")
            else:
                return limit_value_list[0]
        else:
            return original_value