import numpy
from dataclasses import dataclass
from geometry_msgs.msg import *
from tf.transformations import *
from functools import singledispatchmethod

class Translation:
    def __init__(self):
        self.__translation = numpy.zeros(3)

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
    def __init__(self):
        self.__rotation = numpy.zeros((4,4))
    
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
    def __init__(self):
        self.position : Translation = Translation()
        self.orientation : Rotation = Rotation()

class Transform:
    def __init__(self):
        self.linear : Translation = Translation()
        self.angular : Rotation = Rotation()

class Scale:
    def __init__(self, x : float, y : float, z : float):
        self.x : float = x
        self.y : float = y
        self.z : float = z

class Twist:
    linear : Translation
    angular : Rotation

