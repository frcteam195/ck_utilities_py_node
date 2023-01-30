import rospy
from typing import Any, List
import inspect
import dataclasses

def parameter_exists(parameter : str) -> bool:
    return rospy.has_param(param_name="/" + rospy.get_name() + "/" + parameter)

def __load_paramter__(parameter : str, default_value : Any = None, exception_on_failure : bool = True) -> Any:
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

    if not parameter_exists(parameter):
        if (exception_on_failure):
            raise Exception("Missing parameter definition for: " + parameter)
        return default_value
    else:
        if default_value is not None:
            return rospy.get_param(param_name=rospy.get_name() + "/" + parameter, default=default_value)
        else:
            return rospy.get_param(param_name=rospy.get_name() + "/" + parameter)

# def load_parameter_list(parameter_list : List[str], storage_class : Any):
#     for param_string in parameter_list:
#         if not parameter_exists(param_string):
#             raise Exception("Missing parameter definition for: " + param_string)

#         if (not hasattr(storage_class, param_string)):
#             raise Exception("Missing storage class definition for: " + param_string)

#         setattr(storage_class, param_string, __load_paramter__(param_string))

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
            setattr(storage_class_obj, m[0], __load_paramter__(m[0]))