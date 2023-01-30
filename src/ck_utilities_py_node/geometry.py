import numpy
from dataclasses import dataclass

class Translation:
    def __init__(self):
        self.__translation = numpy.zeros(3)
    def x(self) -> float:
        return self.__translation[0]
    def x(self, value : float):
        self.__translation[0] = value
    def y(self) -> float:
        return self.__translation[1]
    def y(self, value : float):
        self.__translation[1] = value
    def z(self) -> float:
        return self.__translation[2]
    def z(self, value : float):
        self.__translation[2] = value

class Rotation:
    def __init__(self):
        self.__rotation = numpy.zeros(3)
    def roll(self) -> float:
        return self.__rotation[0]
    def roll(self, value : float):
        self.__rotation[0] = value
    def pitch(self) -> float:
        return self.__rotation[1]
    def pitch(self, value : float):
        self.__rotation[1] = value
    def yaw(self) -> float:
        return self.__rotation[2]
    def yaw(self, value : float):
        self.__rotation[2] = value

class RotationQuaternion:
    def __init__(self):
        self.__rotation = numpy.zeros(4,4)
    def x(self) -> float:
        return self.__rotation[0,0]
    def x(self, value : float):
        self.__rotation[0,0] = value
    def y(self) -> float:
        return self.__rotation[1,1]
    def y(self, value : float):
        self.__rotation[1,1] = value
    def z(self) -> float:
        return self.__rotation[2,2]
    def z(self, value : float):
        self.__rotation[2,2] = value
    def w(self) -> float:
        return self.__rotation[3,3]
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

