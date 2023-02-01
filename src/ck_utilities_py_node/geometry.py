import numpy
from geometry_msgs.msg import *
from tf.transformations import *

class Translation:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__translation = numpy.zeros(3)
            return
        elif isinstance(input_type, geometry_msgs.msg._Vector3.Vector3):
            self.__init_from_translation(input_type)
            return
        elif isinstance(input_type, geometry_msgs.msg._Point.Point):
            self.__init_from_point(input_type)
        raise ValueError("Type " + str(type(input_type)) + "is not supported by Translation constructor")

    def __init_from_translation(self, input_translation : geometry_msgs.msg._Vector3.Vector3):
        self.__translation = numpy.zeros(3)
        self.x = input_translation.x
        self.y = input_translation.y
        self.z = input_translation.z

    def __init_from_point(self, input_point : geometry_msgs.msg._Point.Point):
        self.__translation = numpy.zeros(3)
        self.x = input_point.x
        self.y = input_point.y
        self.z = input_point.z

    @property
    def x(self):
        return self.__translation[0]
    @x.setter
    def x(self, value):
        self.__translation[0] = value

    @property
    def y(self):
        return self.__translation[1]
    @y.setter
    def y(self, value):
        self.__translation[1] = value

    @property
    def z(self):
        return self.__translation[2]
    @z.setter
    def z(self, value):
        self.__translation[2] = value

class Rotation:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__rotation = numpy.zeros(3)
            return
        elif isinstance(input_type, geometry_msgs.msg._Quaternion.Quaternion):
            self.__init_from_quaternion(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by Rotation constructor")

    def __init_from_quaternion(self, input_quaternion : geometry_msgs.msg._Quaternion.Quaternion):
        self.__rotation = numpy.zeros(3)
        quat = ([input_quaternion.x, input_quaternion.y, input_quaternion.z, input_quaternion.w])
        eulers = euler_from_quaternion(quat)
        self.roll = eulers[0]
        self.yaw = eulers[1]
        self.pitch = eulers[2]

    @property
    def roll(self) -> float:
        return self.__rotation[0]
    @roll.setter
    def roll(self, value : float):
        self.__rotation[0] = value

    @property
    def pitch(self) -> float:
        return self.__rotation[1]
    @pitch.setter
    def pitch(self, value : float):
        self.__rotation[1] = value

    @property
    def yaw(self) -> float:
        return self.__rotation[2]
    @yaw.setter
    def yaw(self, value : float):
        self.__rotation[2] = value

class RotationQuaternion:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__rotation = numpy.zeros(4,4)
            return
        elif isinstance(input_type, geometry_msgs.msg._Quaternion.Quaternion):
            self.__init_from_quaternion(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_quaternion(self, input_quaternion : geometry_msgs.msg._Quaternion.Quaternion):
        self.__rotation = numpy.zeros(4,4)
        self.x = input_quaternion.x
        self.y = input_quaternion.y
        self.z = input_quaternion.z
        self.w = input_quaternion.w

    @property
    def x(self) -> float:
        return self.__rotation[0,0]
    @x.getter
    def x(self, value : float):
        self.__rotation[0,0] = value

    @property
    def y(self) -> float:
        return self.__rotation[1,1]
    @y.getter
    def y(self, value : float):
        self.__rotation[1,1] = value

    @property
    def z(self) -> float:
        return self.__rotation[2,2]
    @z.getter
    def z(self, value : float):
        self.__rotation[2,2] = value

    @property
    def w(self) -> float:
        return self.__rotation[3,3]
    @w.getter
    def w(self, value : float):
        self.__rotation[3,3] = value

class Pose:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__position : Translation = Translation()
            self.__orientation : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._Pose.Pose):
            self.__init_from_pose(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_pose : geometry_msgs.msg._Pose.Pose):
        self.__position = Translation(input_pose.position)
        self.__orientation = Rotation(input_pose.orientation)

    @property
    def position(self) -> Translation:
        return self.__position
    @position.getter
    def position(self, value : Translation):
        self.__position = value

    @property
    def orientation(self) -> Rotation:
        return self.__orientation
    @orientation.getter
    def position(self, value : Rotation):
        self.__orientation = value

class Transform:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__linear : Translation = Translation()
            self.__angular : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._Transform.Transform):
            self.__init_from_pose(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_transform : geometry_msgs.msg._Transform.Transform):
        self.__linear = Translation(input_transform.translation)
        self.__angular = Rotation(input_transform.rotation)

    @property
    def linear(self) -> Translation:
        return self.__linear
    @linear.getter
    def linear(self, value : Translation):
        self.__linear = value

    @property
    def angular(self) -> Rotation:
        return self.__angular
    @angular.getter
    def angular(self, value : Rotation):
        self.__angular = value


class Twist:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__linear : Translation = Translation()
            self.__angular : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._Transform.Transform):
            self.__init_from_pose(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_transform : geometry_msgs.msg._Transform.Transform):
        self.__linear = Translation(input_transform.translation)
        self.__angular = Rotation(input_transform.rotation)

    @property
    def linear(self) -> Translation:
        return self.__linear
    @linear.getter
    def linear(self, value : Translation):
        self.__linear = value

    @property
    def angular(self) -> Rotation:
        return self.__angular
    @angular.getter
    def angular(self, value : Rotation):
        self.__angular = value


class Scale:
    def __init__(self, x : float, y : float, z : float):
        self.x : float = x
        self.y : float = y
        self.z : float = z