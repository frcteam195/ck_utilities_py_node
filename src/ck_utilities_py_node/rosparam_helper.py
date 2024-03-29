"""
A helper to load parameters from the ROS parameter server more easily.

Author: Robert Hilton (robert.a.hilton.jr@gmail.com), Team 195
"""

import rospy
from typing import Any
import inspect
import dataclasses

def __parameter_exists__(parameter : str) -> bool:
    return rospy.has_param(param_name="/" + rospy.get_name() + "/" + parameter)


def __load_parameter__(parameter : str, default_value : Any = None, exception_on_failure : bool = True) -> Any:
    """Checks if a parameter exists on the server and loads it if available, otherwise throws an exception.

    Parameters
    ----------
    parameter : str
        The parameter string to load for the current node
    default_value : Any
        The default value the parameter should be if it does not exist (must also set exception_on_failure to False)
    exception_on_failure : bool
        Whether or not to throw an exception on failure or to just return none. Default is to except.

    Returns
    -------
    Any
        Any type that is returned from the parameter server or potentially None on failure
    """

    if not __parameter_exists__(parameter):
        if (exception_on_failure):
            raise Exception("Missing parameter definition for: " + parameter)
        return default_value
    else:
        if default_value is not None:
            return rospy.get_param(param_name=rospy.get_name() + "/" + parameter, default=default_value)
        else:
            return rospy.get_param(param_name=rospy.get_name() + "/" + parameter)


def load_parameter_class(storage_class_obj : Any) -> None:
    """Loads data into a dataclass object from the parameter server. Class field names must match parameter names exactly.

    This method will load data into the storage dataclass from the parameter server. It will verify that all fields exist within the parameter server
    and will throw an exception if they do not exist.

    Parameters
    ----------
    storage_class_obj : Any
        The dataclass to load parameters for
    """

    if not dataclasses.is_dataclass(storage_class_obj):
        raise Exception("Storage class for parameter loading is not a dataclass in " + rospy.get_name())

    for m in inspect.getmembers(storage_class_obj):
        if not m[0].startswith('_') and not inspect.ismethod(m[1]):
            setattr(storage_class_obj, m[0], __load_parameter__(m[0]))