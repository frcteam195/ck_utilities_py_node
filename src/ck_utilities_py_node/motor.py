#!/usr/bin/env python3

import rospy
from dataclasses import dataclass
from threading import Thread, Lock
from ck_ros_base_msgs_node.msg import Motor_Status
from ck_ros_base_msgs_node.msg import Motor_Control
from ck_ros_base_msgs_node.msg import Motor_Configuration
from ck_ros_base_msgs_node.msg import Motor_Config
import ck_ros_base_msgs_node.msg
from ck_ros_base_msgs_node.msg import Current_Limit_Configuration
from enum import Enum

class NeutralMode(Enum):
    Coast = 1
    Brake = 2

class ConfigMode(Enum):
    Master = 0
    FastMaster = 1
    Follower = 2

class InvertType(Enum):
    Nope = 0
    InvertMotorOutput = 1
    FollowMaster = 2
    OpposeMaster = 3

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

class MotorType(Enum):
    TalonFX = 0
    TalonSRX = 1

class ControlMode(Enum):
    PERCENT_OUTPUT = 0
    POSITION = 1
    VELOCITY = 2
    CURRENT = 3
    __FOLLOWER = 5
    MOTION_PROFILE = 6
    MOTION_MAGIC = 7
    MOTION_PROFILE_ARC = 10
    MUSIC_TONE = 13
    DISABLED = 15

@dataclass
class OutputControl:
    type : MotorType = MotorType.TalonFX
    output : float = 0
    arbFF : float = 0
    controlMode : ControlMode = ControlMode.PERCENT_OUTPUT
@dataclass
class MotorConfig:
    type : MotorType = MotorType.TalonFX
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
        self.__motorConfigs = {}
        self.__motorControls = {}
        self.__motorStatuses = {}
        self.__controlPublisher = rospy.Publisher(name='MotorControl', data_class=Motor_Control, queue_size=50, tcp_nodelay=True)
        self.__configPublisher = rospy.Publisher(name='MotorConfiguration', data_class=Motor_Configuration, queue_size=50, tcp_nodelay=True)
        self.__mutex = Lock()
        x = Thread(target=self.__motorMasterLoop)
        rospy.Subscriber("/MotorStatus", Motor_Status, self.__receive_motor_status)
        x.start()

    def __receive_motor_status(self, data):
        with self.__mutex:
            for motor in data.motors:
                self.__motorStatuses[motor.id] = motor

    def apply_motor_config(self, motorId : int, motorConfig : MotorConfig):
        with self.__mutex:
            self.__motorConfigs[motorId] = motorConfig

    def update_motor_control(self, motorId : int, outputControl : OutputControl):
        with self.__mutex:
            self.__motorControls[motorId] = outputControl
            self.__set_motor_now(motorId, outputControl)

    def get_status(self, id):
        with self.__mutex:
            if id in self.__motorStatuses:
                return self.__motorStatuses[id]
            return None

    @staticmethod
    def __create_motor_control_dictionary(motorId : int, motorControl : OutputControl):
        motorControlMsg = ck_ros_base_msgs_node.msg.Motor()
        motorControlMsg.id = motorId
        motorControlMsg.controller_type = motorControl.type.value
        motorControlMsg.control_mode = motorControl.controlMode.value
        motorControlMsg.output_value = motorControl.output
        motorControlMsg.arbitrary_feedforward = motorControl.arbFF
        return motorControlMsg

    @staticmethod
    def __create_motor_config_dictionary(motorId : int, motorConfig : MotorConfig):
        motorConfigMsg = Motor_Config()
        motorConfigMsg.id = motorId
        motorConfigMsg.controller_type = motorConfig.type.value
        if motorConfig.followingEnabled:
            motorConfigMsg.controller_mode = ConfigMode.Follower.value
            motorConfigMsg.invert_type = InvertType.OpposeMaster.value if motorConfig.inverted else InvertType.FollowMaster.value
        elif motorConfig.fast_master:
            motorConfigMsg.controller_mode = ConfigMode.FastMaster.value
            motorConfigMsg.invert_type = InvertType.InvertMotorOutput.value if motorConfig.inverted else InvertType.Nope.value
        else:
            motorConfigMsg.controller_mode = ConfigMode.Master.value
            motorConfigMsg.invert_type = InvertType.InvertMotorOutput.value if motorConfig.inverted else InvertType.Nope.value
        motorConfigMsg.kP = motorConfig.kP
        motorConfigMsg.kI = motorConfig.kI
        motorConfigMsg.kD = motorConfig.kD
        motorConfigMsg.kF = motorConfig.kF
        motorConfigMsg.iZone = motorConfig.iZone
        motorConfigMsg.max_i_accum = motorConfig.maxIAccum
        motorConfigMsg.allowed_closed_loop_error = motorConfig.allowedClosedLoopError
        motorConfigMsg.max_closed_loop_peak_output = motorConfig.maxClosedLoopPeakOutput
        motorConfigMsg.motion_cruise_velocity = motorConfig.motionCruiseVelocity
        motorConfigMsg.motion_acceleration = motorConfig.motionCruiseAcceleration
        motorConfigMsg.motion_s_curve_strength = motorConfig.motionSCurveStrength
        motorConfigMsg.forward_soft_limit = motorConfig.forwardSoftLimit
        motorConfigMsg.forward_soft_limit_enable = motorConfig.forwardSoftLimitEnable
        motorConfigMsg.reverse_soft_limit = motorConfig.reverseSoftLimit
        motorConfigMsg.reverse_soft_limit_enable = motorConfig.reverseSoftLimitEnable
        motorConfigMsg.feedback_sensor_coefficient = motorConfig.feedbackSensorCoefficient
        motorConfigMsg.voltage_compensation_saturation = motorConfig.voltageCompensationSaturation
        motorConfigMsg.voltage_compensation_enabled = motorConfig.voltageCompensationEnabled
        motorConfigMsg.sensor_phase_inverted = motorConfig.sensorPhaseInverted
        motorConfigMsg.neutral_mode = motorConfig.neutralMode.value
        motorConfigMsg.open_loop_ramp = motorConfig.openLoopRamp
        motorConfigMsg.closed_loop_ramp = motorConfig.closedLoopRamp
        supplyCurrentLimit = Current_Limit_Configuration()
        supplyCurrentLimit.enable = motorConfig.supplyCurrentLimitEnable
        supplyCurrentLimit.current_limit = motorConfig.supplyCurrentLimit
        supplyCurrentLimit.trigger_threshold_current = motorConfig.supplyCurrentLimitThresholdCurrent
        supplyCurrentLimit.trigger_threshold_time = motorConfig.supplyCurrentLimitThresholdTime
        motorConfigMsg.supply_current_limit_config = supplyCurrentLimit
        statorCurrentLimit = Current_Limit_Configuration()
        statorCurrentLimit.enable = motorConfig.statorCurrentLimitEnable
        statorCurrentLimit.current_limit = motorConfig.statorCurrentLimit
        statorCurrentLimit.trigger_threshold_current = motorConfig.statorCurrentLimitThresholdCurrent
        statorCurrentLimit.trigger_threshold_time = motorConfig.statorCurrentLimitThresholdTime
        motorConfigMsg.stator_current_limit_config = statorCurrentLimit
        motorConfigMsg.forward_limit_switch_source = motorConfig.forwardLimitSwitchSource.value
        motorConfigMsg.forward_limit_switch_normal = motorConfig.forwardLimitSwitchNormal.value
        motorConfigMsg.reverse_limit_switch_source = motorConfig.reverseLimitSwitchSource.value
        motorConfigMsg.reverse_limit_switch_normal = motorConfig.reverseLimitSwitchNormal.value
        motorConfigMsg.peak_output_forward = motorConfig.peakOutputForward
        motorConfigMsg.peak_output_reverse = motorConfig.peakOutputReverse
        return motorConfigMsg

    def __transmit_motor_configs(self):
        configMessage = Motor_Configuration()
        configMessage.motors = []
        for motorId in self.__motorConfigs.keys():
            if motorId in self.__motorControls:
                configMessage.motors.append(self.__create_motor_config_dictionary(motorId, self.__motorConfigs[motorId]))
        self.__configPublisher.publish(configMessage)

    def __transmit_motor_controls(self):
        controlMessage = Motor_Control()
        controlMessage.motors = []
        for motorId in self.__motorControls.keys():
            if motorId in self.__motorConfigs:
                if self.__motorConfigs[motorId]:
                    controlStructure = self.__motorControls[motorId]
                    if self.__motorConfigs[motorId].followingEnabled:
                        controlStructure.controlMode = ControlMode.__FOLLOWER
                        controlStructure.output = self.__motorConfigs[motorId].followerId
                controlMessage.motors.append(self.__create_motor_control_dictionary(motorId, controlStructure))
        self.__controlPublisher.publish(controlMessage)

    def __set_motor_now(self, motorId : int, outputControl : OutputControl):
        controlMessage = Motor_Control()
        controlMessage.motors = []
        controlMessage.motors.append(self.__create_motor_control_dictionary(motorId, self.__motorControls[motorId]))
        self.__controlPublisher.publish(controlMessage)

    def __motorMasterLoop(self):
        r = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            with self.__mutex:
                self.__transmit_motor_controls()
                self.__transmit_motor_configs()
            r.sleep()

class Motor:
    manager = None
    mutex = Lock()

    def __init__(self, id : int, type : MotorType):
        self.config = MotorConfig()
        self.config.type = type
        self.id = id
        self.type = type
        self.spawn_motor_manager()

    @classmethod
    def spawn_motor_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = MotorManager()

    def apply(self):
        with self.__class__.mutex:
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
        self.config.forwardSoftLimitEnable = True
        self.config.forwardSoftLimit = value

    def set_forward_soft_limit_enable(self, enabled : bool):
        self.config.forwardSoftLimitEnable = enabled

    def set_reverse_soft_limit(self, value : float):
        self.config.reverseSoftLimitEnable = True
        self.config.reverseSoftLimit = value

    def set_reverse_soft_limit_enable(self, enabled : bool):
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
        self.config.type = self.type

    def set(self, controlMode : ControlMode, output : float, arbitraryFeedForward : float):
        outputControl = OutputControl()
        outputControl.controlMode = controlMode
        outputControl.output = output
        outputControl.arbFF = arbitraryFeedForward
        outputControl.type = self.type
        with self.__class__.mutex:
            self.__class__.manager.update_motor_control(self.id, outputControl)

    def __get_status(self):
        with self.__class__.mutex:
            return __class__.manager.get_status(self.id)

    def get_sensor_position(self):
        status = self.__get_status()
        if status is not None:
            return status.sensor_position
        return 0.0

    def get_sensor_velocity(self):
        status = self.__get_status()
        if status is not None:
            return status.sensor_velocity
        return 0.0

    def get_bus_voltage(self):
        status = self.__get_status()
        if status is not None:
            return status.bus_voltage
        return 0.0

    def get_bus_current(self):
        status = self.__get_status()
        if status is not None:
            return status.bus_current
        return 0.0

    def get_stator_current(self):
        status = self.__get_status()
        if status is not None:
            return status.stator_current
        return 0.0

    def get_forward_limit_closed(self):
        status = self.__get_status()
        if status is not None:
            return status.forward_limit_closed
        return False

    def get_reverse_limit_closed(self):
        status = self.__get_status()
        if status is not None:
            return status.reverse_limit_closed
        return False

    def get_control_mode(self):
        status = self.__get_status()
        if status is not None:
            return status.control_mode
        return 0

    def get_commanded_output(self):
        status = self.__get_status()
        if status is not None:
            return status.commanded_output
        return 0.0

    def get_active_trajectory_arbff(self):
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_arbff
        return 0.0

    def get_active_trajectory_position(self):
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_position
        return 0.0

    def get_active_trajectory_velocity(self):
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_velocity
        return 0.0

    def get_raw_closed_loop_error(self):
        status = self.__get_status()
        if status is not None:
            return status.raw_closed_loop_error
        return 0.0

    def get_raw_integral_accum(self):
        status = self.__get_status()
        if status is not None:
            return status.raw_integral_accum
        return 0.0

    def get_raw_error_derivative(self):
        status = self.__get_status()
        if status is not None:
            return status.raw_error_derivative
        return 0.0

    def get_raw_output_percent(self):
        status = self.__get_status()
        if status is not None:
            return status.raw_output_percent
        return 0.0
