from rev import SparkMaxConfig, FeedbackSensor, AbsoluteEncoderConfig

from constants import ModuleConstants

from math import pi


class HopperConfigs:
    pass


class IntakeConfigs:
    pass


class ShooterConfigs:
    pass


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
        AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder
    )
    turning_motor_config.closedLoop.setFeedbackSensor(
        FeedbackSensor.kAbsoluteEncoder
    ).pid(1.0, 0.0, 0.0).outputRange(-1.0, 1.0).positionWrappingEnabled(
        True
    ).positionWrappingInputRange(
        0, turning_factor
    )
