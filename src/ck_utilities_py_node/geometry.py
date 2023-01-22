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

@dataclass
class Pose:
    position : Translation
    orientation : Rotation

class Transform:
    def __init__(self):
        self.linear = Translation()
        self.angular = Rotation()

@dataclass
class Twist:
    linear : Translation
    angular : Rotation

@dataclass
class TransformLink:
    base_frame : str
    translation : Transform
