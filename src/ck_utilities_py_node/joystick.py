import rospy
from dataclasses import dataclass
from threading import Thread, Lock
from enum import Enum
from ck_ros_base_msgs_node.msg import Joystick_Status
from ck_utilities_py_node.ckmath import *
from typing import Tuple

def MAX_NUM_JOYSTICKS() -> int :
    return 6

def MAX_NUM_AXES() -> int :
    return 8

def MAX_NUM_BUTTONS() -> int :
    return 16

def MAX_NUM_POVS() -> int :
    return 4

class JoystickIDOutOfRangeException(Exception):
    pass

class Joystick:
    mutex = Lock()
    joystick_map = {}

    def __init__(self, id : int):
        self.__id = id
        self.__prevButtonVals = {}
        self.__prevPOVVals = {}
        if id > MAX_NUM_JOYSTICKS():
            raise JoystickIDOutOfRangeException("Joystick ID " + str(id) + " out of range!")

    @classmethod
    def update(cls, msg : Joystick_Status):
        with cls.mutex:
            for joystick in msg.joysticks:
                cls.joystick_map[joystick.index] = joystick

    def getRawAxis(self, axisID : int) -> int:
        if self.__id in self.joystick_map:
            if axisID < MAX_NUM_AXES() and len(self.joystick_map[self.__id].axes) > axisID:
                return self.joystick_map[self.__id].axes[axisID]
        return 0

    def getFilteredAxis(self, axisID : int, deadband : float) -> float:
        return normalizeWithDeadband(self.getRawAxis(axisID), deadband)

    def getAxisActuated(self, axisID : int, threshold : float) -> bool:
        return self.getRawAxis(axisID) > threshold

    def getButton(self, buttonID : int) -> bool:
        if buttonID < MAX_NUM_BUTTONS():
            retVal = False
            if self.__id in self.joystick_map:
                if len(self.joystick_map[self.__id].buttons) > buttonID:
                    retVal = self.joystick_map[self.__id].buttons[buttonID]
            self.__prevButtonVals[buttonID] = retVal
            return retVal
        return False

    def getRisingEdgeButton(self, buttonID : int) -> bool:
        if buttonID < MAX_NUM_BUTTONS():
            currVal = False
            if self.__id in self.joystick_map:
                if len(self.joystick_map[self.__id].buttons) > buttonID:
                    currVal = self.joystick_map[self.__id].buttons[buttonID]
            retVal = currVal and (currVal != self.__prevButtonVals[buttonID])
            self.__prevButtonVals[buttonID] = currVal
            return retVal
        return False

    def getFallingEdgeButton(self, buttonID : int) -> bool:
        if buttonID < MAX_NUM_BUTTONS():
            currVal = False
            if self.__id in self.joystick_map:
                if len(self.joystick_map[self.__id].buttons) > buttonID:
                    currVal = self.joystick_map[self.__id].buttons[buttonID]
            retVal = (not currVal) and (currVal != self.__prevButtonVals[buttonID])
            self.__prevButtonVals[buttonID] = currVal
            return retVal
        return False

    def getPOV(self, povID : int) -> int:
        retVal = -1
        if self.__id in self.joystick_map:
            if povID < MAX_NUM_POVS() and len(self.joystick_map[self.__id].povs) > povID:
                retVal = self.joystick_map[self.__id].povs[povID]
            #self.__prevPOVVals[povID] = retVal
        return retVal
            
    def getRisingEdgePOV(self, povID : int) -> Tuple[bool, int]:
        retVal_bool = False
        retVal_povint = -1
        if self.__id in self.joystick_map:
            if povID < MAX_NUM_POVS() and len(self.joystick_map[self.__id].povs) > povID:
                retVal_povint = self.joystick_map[self.__id].povs[povID]
                retVal_bool = (retVal_povint >= -1) and (retVal_povint != self.__prevPOVVals[povID])
                self.__prevPOVVals[povID] = retVal_povint
        return retVal_bool, retVal_povint
