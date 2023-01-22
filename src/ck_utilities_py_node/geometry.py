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
    def __init__(self):
        self.x : float = 1
        self.y : float = 1
        self.z : float = 1

@dataclass
class Twist:
    linear : Translation
    angular : Rotation

