from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VelocityVoltage
from wpilib import RobotBase
from configs import ShooterConfigs
from constants import CANConstants, ShooterConstants, FieldConstants
from configs import ShooterConfigs
from state_system import *
from math import pi, sin
from wpimath.geometry import Pose2d

from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class ShooterSubsystem(StateSystem):
    upper_roller_motor = TalonFX(CANConstants.upper_roller_motor)
    lower_roller_motor = TalonFX(CANConstants.lower_roller_motor)
    conveyor_motor = TalonFX(CANConstants.conveyor_motor)
    trigger_motor = TalonFX(CANConstants.trigger_motor)

    idle_shooter_rps = 20
    target_shooter_rps: float | None = None
    outtaking = False

    def __init__(self, robot_drive: SwerveDriveSubsystem):
        # Initialize the state machine
        super().__init__()

        self.robot_drive = robot_drive

        self.upper_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.lower_roller_motor.setNeutralMode(NeutralModeValue.COAST)
        self.conveyor_motor.setNeutralMode(NeutralModeValue.COAST)
        self.trigger_motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.upper_roller_motor.configurator.apply(ShooterConfigs.roller_config)
        self.lower_roller_motor.configurator.apply(ShooterConfigs.roller_config)
        self.conveyor_motor.configurator.apply(ShooterConfigs.roller_config)
        self.trigger_motor.configurator.apply(ShooterConfigs.roller_config)

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

        if self.target_shooter_rps:
            target_rps: float = self.target_shooter_rps
        else:
            target_rps: float = self.idle_shooter_rps

        if not self.target_shooter_rps and not self.outtaking:
            time = wpilib.Timer.getFPGATimestamp() * 7.5 * pi
            power_applied: float = (
                (self.sign(sin(time)) * (abs(sin(time)) ** 1.2) + 0.1 * sin(10 * time))
                / 11
                * 2.5
            )

            self.trigger_motor.set(power_applied)
            self.conveyor_motor.set(power_applied)

        robot_pose: Pose2d = self.robot_drive.get_pose()

        in_center: bool = (
            FieldConstants.blue_alliance_end
            < robot_pose.X()
            < FieldConstants.red_alliance_start
        )

        upper_multiplier: float = (
            ShooterConstants.topspin_multiplier
            if in_center
            else ShooterConstants.backspin_correction_multiplier
        )
        lower_multiplier: float = (
            ShooterConstants.topspin_correction_multiplier
            if in_center
            else ShooterConstants.backspin_multiplier
        )

        self.upper_roller_motor.set_control(
            VelocityVoltage(target_rps * upper_multiplier)
        )
        self.lower_roller_motor.set_control(
            VelocityVoltage(-target_rps * lower_multiplier)
        )

    @staticmethod
    def sign(x: float) -> float:
        if x > 0:
            return 1.0
        elif x < 0:
            return -1.0
        else:
            return 0.0

    @state
    def start_conveyor(self):
        self.conveyor_motor.set_control(VelocityVoltage(-20))
        return True

    @state
    def init_shooter(self):
        self.trigger_motor.set(0)
        self.conveyor_motor.set(0)
        self.target_shooter_rps = ShooterConstants.get_shooter_rpm(
            FieldConstants.get_hub_dist(self.robot_drive.get_pose())
        )
        return True

    @state
    def ensure_velocity(self):
        target_rps = (
            self.target_shooter_rps
            if self.target_shooter_rps
            else self.idle_shooter_rps
        )

        robot_pose: Pose2d = self.robot_drive.get_pose()

        in_center: bool = (
            FieldConstants.blue_alliance_end
            < robot_pose.X()
            < FieldConstants.red_alliance_start
        )

        upper_multiplier: float = (
            ShooterConstants.topspin_multiplier
            if in_center
            else ShooterConstants.backspin_correction_multiplier
        )
        lower_multiplier: float = (
            ShooterConstants.topspin_correction_multiplier
            if in_center
            else ShooterConstants.backspin_multiplier
        )

        return_condition = (
            abs(
                self.upper_roller_motor.get_velocity().value_as_double
                - target_rps * upper_multiplier
            )
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(
                self.lower_roller_motor.get_velocity().value_as_double
                + target_rps * lower_multiplier
            )
            < ShooterConstants.minimum_acceptable_closed_loop_error
        )

        return return_condition

    @state
    def advance_balls(self):
        self.trigger_motor.set_control(
            VelocityVoltage(ShooterConstants.advancement_motor_rps)
        )
        self.conveyor_motor.set_control(
            VelocityVoltage(ShooterConstants.conveyor_motor_rps)
        )
        return True

    @state
    def shoot(self):
        self.target_shooter_rps = ShooterConstants.get_shooter_rpm(
            FieldConstants.get_hub_dist(self.robot_drive.get_pose())
        )
        return False

    def set_intake_roller_speed(self):
        self.trigger_motor.set_control(
            VelocityVoltage(ShooterConstants.advancement_motor_rps)
        )
        self.conveyor_motor.set_control(
            VelocityVoltage(ShooterConstants.conveyor_motor_rps)
        )

    def outtake(self):
        self.outtaking = True

        self.trigger_motor.set_control(
            VelocityVoltage(-ShooterConstants.conveyor_motor_rps)
        )
        self.conveyor_motor.set_control(
            VelocityVoltage(-ShooterConstants.conveyor_motor_rps)
        )

    def disable_shooter(self):
        self.upper_roller_motor.stopMotor()
        self.lower_roller_motor.stopMotor()
        self.conveyor_motor.stopMotor()
        self.trigger_motor.stopMotor()
        self.target_shooter_rps = None
        self.outtaking = False
        self.clear_queue()
