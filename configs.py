from rev import SparkMaxConfig, FeedbackSensor, AbsoluteEncoderConfig

from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import MotorArrangementValue

from constants import ModuleConstants

from math import pi


class HopperConfigs:
    hopper_motor_config = TalonFXSConfiguration()
    hopper_slot0 = hopper_motor_config.slot0
    motion_magic_configs = hopper_motor_config.motion_magic
    limit_configs = hopper_motor_config.current_limits

    hopper_motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST

    hopper_slot0.k_p = 0.65
    hopper_slot0.k_i = 0.15
    hopper_slot0.k_d = 0.0

    motion_magic_configs.motion_magic_cruise_velocity = 1600
    motion_magic_configs.motion_magic_acceleration = 2400
    motion_magic_configs.motion_magic_jerk = 3600

    limit_configs.supply_current_limit_enable = True
    limit_configs.supply_current_limit = 17.5


class IntakeConfigs:
    intake_motor_config = TalonFXConfiguration()
    intake_slot0 = intake_motor_config.slot0
    limit_configs = intake_motor_config.current_limits

    intake_slot0.k_p = 0.1
    intake_slot0.k_i = 0.0
    intake_slot0.k_d = 0.0

    limit_configs.supply_current_limit_enable = True
    limit_configs.supply_current_limit = 50


class ShooterConfigs:
    roller_config = TalonFXConfiguration()
    intake_slot0 = roller_config.slot0

    intake_slot0.k_s = 0.18
    intake_slot0.k_v = 0.12
    intake_slot0.k_p = 0.05
    intake_slot0.k_i = 0.003
    intake_slot0.k_d = 0.01


class DriveConfigs:
    drive_motor_config: SparkMaxConfig = SparkMaxConfig()
    turning_motor_config: SparkMaxConfig = SparkMaxConfig()

    driving_factor: float = (
        ModuleConstants.wheel_circumference_m / ModuleConstants.driving_motor_reduction
    )
    turning_factor: float = 2 * pi
    nominal_voltage: float = 12.0
    driving_velocity_feed_forward: float = (
        nominal_voltage / ModuleConstants.drive_wheel_free_speed_rps
    )

    drive_motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(50)
    drive_motor_config.encoder.positionConversionFactor(
        driving_factor
    ).velocityConversionFactor(driving_factor / 60.0)
    drive_motor_config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        0.1, 0.0, 0.0
    ).outputRange(-1.0, 1.0).feedForward.kV(driving_velocity_feed_forward)

    turning_motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(
        20
    )
    turning_motor_config.absoluteEncoder.inverted(True).positionConversionFactor(
        turning_factor
    ).velocityConversionFactor(turning_factor / 60.0).apply(
        AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder()
    )
    turning_motor_config.closedLoop.setFeedbackSensor(
        FeedbackSensor.kAbsoluteEncoder
    ).pid(1.0, 0.0, 0.0).outputRange(-1.0, 1.0).positionWrappingEnabled(
        True
    ).positionWrappingInputRange(
        0, turning_factor
    )
