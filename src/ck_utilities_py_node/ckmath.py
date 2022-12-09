import numpy as np

def handleDeadband(val, deadband):
    return val if abs(val) > abs(deadband) else 0

def normalizeWithDeadband(val, deadband):
    val = handleDeadband(val, deadband)
    if val != 0:
        val = np.sign(val) * ((abs(val) - deadband) / (1.0 - deadband))
    return val