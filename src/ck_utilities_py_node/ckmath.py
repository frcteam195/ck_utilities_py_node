import numpy as np
import math


def handleDeadband(val, deadband):
    return val if abs(val) > abs(deadband) else 0


def normalizeWithDeadband(val, deadband):
    val = handleDeadband(val, deadband)
    if val != 0:
        val = np.sign(val) * ((abs(val) - deadband) / (1.0 - deadband))
    return val


def within(value1, value2, threshold) -> bool:
    return True if abs(value1 - value2) < abs(threshold) else False


def hypotenuse(x, y):
    return np.sqrt((x*x)+(y*y))


def wrapMax(x, max):
    return np.fmod(max + np.fmod(x, max), max)


def wrapMinMax(x, min, max):
    return min + wrapMax(x - min, max - min)


def normalize_to_2_pi(value):
    return wrapMinMax(value, 0, (2.0 * math.pi))


def polar_angle_rad(x, y):
    return normalize_to_2_pi(np.arctan2(y, x))

def limit(v, minVal, maxVal):
    return min(maxVal, max(minVal, v))
        
