#!/usr/bin/env python3

from typing import List
import rospy
from dataclasses import dataclass
from threading import Thread, Lock
from ck_ros_base_msgs_node.msg import Solenoid_Control
from ck_ros_base_msgs_node.msg import Solenoid as SolenoidROS
from enum import Enum

class SolenoidType(Enum):
    SINGLE = 0
    DOUBLE = 1

class SolenoidState(Enum):
    OFF = 0
    ON = 1
    FORWARD = 1
    REVERSE = 2

class __ModuleType__(Enum):
    CTREPCM = 0
    REVPH = 1

@dataclass
class SolenoidControl:
    id : int = 0
    type : SolenoidType = SolenoidType.SINGLE
    output : SolenoidState = SolenoidState.OFF

class SolenoidManager:
    def __init__(self):
        self.__solenoidControls : dict[int, SolenoidControl] = {}
        self.__controlPublisher = rospy.Publisher(name='SolenoidControl', data_class=Solenoid_Control, queue_size=50, tcp_nodelay=True)
        self.__mutex = Lock()
        x = Thread(target=self.__solenoidMasterLoop)
        x.start()

    @staticmethod
    def __create_solenoid_control_dictionary(solenoidId : int, solenoidControl : SolenoidControl):
        solenoidControlMsg = SolenoidROS()
        solenoidControlMsg.id = solenoidId
        solenoidControlMsg.solenoid_type = solenoidControl.type.value
        solenoidControlMsg.module_type = __ModuleType__.CTREPCM.value
        solenoidControlMsg.output_value = solenoidControl.output.value
        return solenoidControlMsg

    def __transmit_solenoid_controls(self):
        controlMessage = Solenoid_Control()
        controlMessage.solenoids = []
        for solenoid in self.__solenoidControls:
            controlMessage.solenoids.append(self.__create_solenoid_control_dictionary(self.__solenoidControls[solenoid].id, self.__solenoidControls[solenoid]))
        self.__controlPublisher.publish(controlMessage)

    def __set_solenoid_now(self, solenoidId : int, outputControl : SolenoidControl):
        controlMessage = Solenoid_Control()
        controlMessage.solenoids = []
        controlMessage.solenoids.append(self.__create_solenoid_control_dictionary(solenoidId, outputControl))
        self.__controlPublisher.publish(controlMessage)

    def update_solenoid_control(self, solenoidId : int, outputControl : SolenoidControl):
        with self.__mutex:
            self.__solenoidControls[solenoidId] = outputControl
            self.__set_solenoid_now(solenoidId, outputControl)

    def __solenoidMasterLoop(self):
        r = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            with self.__mutex:
                self.__transmit_solenoid_controls()
            r.sleep()

class Solenoid:
    manager : SolenoidManager = None
    mutex = Lock()

    def __init__(self, id : int, type : SolenoidType):
        self.__solenoidControl : SolenoidControl = SolenoidControl()
        self.__solenoidControl.id = id
        self.__solenoidControl.type = type
        self.__spawn_solenoid_manager()

    @classmethod
    def __spawn_solenoid_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = SolenoidManager()

    def __update(self):
        with self.__class__.mutex:
            self.__class__.manager.update_solenoid_control(self.__solenoidControl.id, self.__solenoidControl)

    def set(self, output : SolenoidState):
        self.__solenoidControl.output = output
        self.__update()