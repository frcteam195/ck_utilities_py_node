#!/usr/bin/env python3

import tf2_ros
import rospy
from dataclasses import dataclass
from threading import Thread, Lock
from ck_utilities_py_node.motor import test_function
from rio_control_node.msg import Motor_Status
from rio_control_node.msg import Motor_Control
from enum import Enum

class NeutralMode(Enum):
    Coast = 1
    Brake = 2

class LimitSwitchSource(Enum):
    FeedbackConnector = 0
    RemoteTalon = 1
    RemoteTalonSRX = 1
    RemoteCANifier = 2
    Deactivated = 3

class LimitSwitchNormal(Enum):
    NormallyOpen = 0
    NormallyClosed = 1
    Disabled = 2

@dataclass
class MotorConfig:
    fast_master : bool = False
    kP : float = 0
    kI : float = 0
    kD : float = 0
    kF : float = 0
    iZone : float = 0
    maxIAccum : float = 0
    allowedClosedLoopError : float = 0
    maxClosedLoopPeakOutput : float = 0
    motionCruiseVelocity : float = 0
    motionCruiseAcceleration : float = 0
    motionSCurveStrength : float = 0
    forwardSoftLimit : float = 0
    forwardSoftLimitEnable : bool = False
    reverseSoftLimit : float = 0
    reverseSoftLimitEnable : bool = False
    feedbackSensorCoefficient : float = 0
    voltageCompensationSaturation : float = 12
    voltageCompensationEnabled : bool = True
    inverted : bool = False
    sensorPhaseInverted : bool = False
    neutralMode : NeutralMode = NeutralMode.Coast
    openLoopRamp : float = 0
    closedLoopRamp : float = 0
    supplyCurrentLimitEnable : bool = True
    supplyCurrentLimit : float = 40
    supplyCurrentLimitThresholdCurrent : float = 0
    supplyCurrentLimitThresholdTime : float = 0
    statorCurrentLimitEnable : bool = False
    statorCurrentLimit : float = 0
    statorCurrentLimitThresholdCurrent : float = 0
    statorCurrentLimitThresholdTime : float =  0
    followingEnabled : bool = False
    followerId : int = 0
    forwardLimitSwitchSource : LimitSwitchSource = LimitSwitchSource.Deactivated
    reverseLimitSwitchSource : LimitSwitchSource = LimitSwitchSource.Deactivated
    forwardLimitSwitchNormal : LimitSwitchNormal = LimitSwitchNormal.Disabled
    reverseLimitSwitchNormal : LimitSwitchNormal = LimitSwitchNormal.Disabled
    peakOutputForward : float = 0
    peakOutputReverse : float = 0

class MotorManager:
    def __init__(self):
        print("I was started")

    def apply_motor_config(motor_id : int, motorConfig : MotorConfig):
        print()


class Motor:
    manager = None
    mutex = Lock()

    def __init__(self, id):
        self.config = MotorConfig()
        self.id = id
        print()
        self.spawn_motor_manager()

    @classmethod
    def spawn_motor_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = MotorManager()

    def apply(self):
        __class__.manager.apply_motor_config(self.id, self.config)

    def set_fast_master(self, enable : bool):
        self.config.fast_master = enable

    def set_kP(self, value : float):
        self.config.kP = value

    def set_kI(self, value : float):
        self.config.kI = value

    def set_kD(self, value : float):
        self.config.kD = value

    def set_kF(self, value : float):
        self.config.kF = value

    def set_i_zone(self, value : float):
        self.config.iZone = value

    def set_max_i_accum(self, value : float):
        self.config.maxIAccum = value

    def set_allowed_closed_loop_error(self, value : float):
        self.config.allowedClosedLoopError = value

    def set_max_closed_loop_peak_output(self, value : float):
        self.config.closedLoopRamp = value

    def set_motion_cruise_velocity(self, value : float):
        self.config.motionCruiseVelocity = value

    def set_motion_acceleration(self, value : float):
        self.config.motionCruiseAcceleration = value

    def set_motion_s_curve_strength(self, value : float):
        self.config.motionSCurveStrength = value

    def set_forward_soft_limit(self, value : float):
        self.config.forwardSoftLimit = value

    def set_forward_soft_limit_enable(self, enabled : float):
        self.config.forwardSoftLimitEnable = enabled

    def set_reverse_soft_limit(self, value : float):
        self.config.reverseSoftLimit = value

    def set_reverse_soft_limit_enable(self, enabled : float):
        self.config.reverseSoftLimitEnable = enabled

    def set_feedback_sensor_coefficient(self, value : float):
        self.config.feedbackSensorCoefficient = value

    def set_voltage_compensation_saturation(self, value : float):
        self.config.voltageCompensationSaturation = value

    def set_voltage_compensation_enabled(self, enabled : bool):
        self.config.voltageCompensationEnabled = enabled

    def set_inverted(self, enabled : bool):
        self.config.inverted = enabled

    def set_sensor_phase_inverted(self, enabled : bool):
        self.config.sensorPhaseInverted = enabled

    def set_neutral_mode(self, mode : NeutralMode):
        self.config.neutralMode = mode

    def set_open_loop_ramp(self, value : float):
        self.config.openLoopRamp = value

    def set_closed_loop_ramp(self, value : float):
        self.config.closedLoopRamp = value

    def set_supply_current_limit(self, enabled : bool, current_limit : float, trigger_current : float, trigger_time : float):
        self.config.supplyCurrentLimitEnable = enabled
        self.config.supplyCurrentLimit = current_limit
        self.config.supplyCurrentLimitThresholdCurrent = trigger_current
        self.config.supplyCurrentLimitThresholdTime = trigger_time

    def set_stator_current_limit(self, enabled : bool, current_limit : float, trigger_current : float, trigger_time : float):
        self.config.statorCurrentLimitEnable = enabled
        self.config.statorCurrentLimit = current_limit
        self.config.statorCurrentLimitThresholdCurrent = trigger_current
        self.config.statorCurrentLimitThresholdTime = trigger_time

    def set_follower(self, enabled : bool, master_id : float):
        self.config.followingEnabled = enabled
        self.config.followerId = master_id

    def set_forward_limit_switch(self, forward_limit_switch_source : LimitSwitchSource, forward_limit_switch_normal : LimitSwitchNormal):
        self.config.forwardLimitSwitchSource = forward_limit_switch_source
        self.config.forwardLimitSwitchNormal = forward_limit_switch_normal

    def set_reverse_limit_switch(self, reverse_limit_switch_source : LimitSwitchSource, reverse_limit_switch_normal : LimitSwitchNormal):
        self.config.reverseLimitSwitchSource = reverse_limit_switch_source
        self.config.reverseLimitSwitchNormal = reverse_limit_switch_normal

    def set_peak_output_forward(self, value : float):
        self.config.peakOutputForward = value

    def set_peak_output_reverse(self, value : float):
        self.config.peakOutputReverse = value

    def set_defaults(self):
        self.config = MotorConfig()
