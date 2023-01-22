import numpy as np
from dataclasses import dataclass

@dataclass
class Translation:
    x : float = 0
    y : float = 0
    z : float = 0

@dataclass
class Rotation:
    yaw : float = 0
    pitch : float = 0
    roll : float = 0

@dataclass
class RotationQuaternion:
    x : float = 0
    y : float = 0
    z : float = 0
    w : float = 1

class Pose:
    position : Translation
    orientation : Rotation

class Transform:
    def __init__(self):
        self.linear = Translation()
        self.angular = Rotation()

class Scale:
    def __init__(self, x : float, y : float, z : float):
        self.x : float = x
        self.y : float = y
        self.z : float = z

@dataclass
class Twist:
    linear : Translation
    angular : Rotation

