from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import VelocityVoltage
from configs import ShooterConfigs
from constants import CANConstants, ShooterConstants, FieldConstants
from configs import ShooterConfigs
from state_system import *
from time import sleep

from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class ShooterSubsytem(StateSystem):
    upper_roller_motor = TalonFX(CANConstants.upper_roller_motor)
    lower_roller_motor = TalonFX(CANConstants.lower_roller_motor)
    conveyor_motor = TalonFX(CANConstants.conveyor_motor)
    trigger_motor = TalonFX(CANConstants.trigger_motor)

    idle_shooter_rps = ShooterConstants.get_shooter_rpm(0)
    target_shooter_rps: float | None = None

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
            target_rps: float = ShooterConstants.get_shooter_rpm(FieldConstants.get_hub_dist(self.robot_drive.get_pose()))
        else:
            target_rps: float = self.idle_shooter_rps
        
        self.upper_roller_motor.set_control(
            VelocityVoltage(
                target_rps
            )
        )
        self.lower_roller_motor.set_control(
            VelocityVoltage(
                -target_rps
            )
        )

    @state
    def start_conveyor(self):
        self.conveyor_motor.set_control(VelocityVoltage(-20))
        return True

    @state
    def init_shooter(self):
        self.target_shooter_rps = FieldConstants.get_hub_dist(self.robot_drive.get_pose())
        self.queue_state("ensure_velocity", 0)
        return True

    @state
    def ensure_velocity(self):
        target_rps = self.target_shooter_rps if self.target_shooter_rps else self.idle_shooter_rps

        return_condition = (
            abs(self.upper_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
        )

        if return_condition:
            self.queue_state("advance_balls", 0)

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
        target_rps = ShooterConstants.get_shooter_rpm(FieldConstants.get_hub_dist(self.robot_drive.get_pose()))

        self.upper_roller_motor.set_control(VelocityVoltage(target_rps))
        self.lower_roller_motor.set_control(VelocityVoltage(-target_rps))

        if not (
            abs(self.upper_roller_motor.get_velocity().value_as_double - target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
            and abs(self.lower_roller_motor.get_velocity().value_as_double + target_rps)
            < ShooterConstants.minimum_acceptable_closed_loop_error
        ):
            return False

        self.set_intake_roller_speed()

        return False

    def set_intake_roller_speed(self):
        self.trigger_motor.set_control(
            VelocityVoltage(ShooterConstants.advancement_motor_rps)
        )
        self.conveyor_motor.set_control(
            VelocityVoltage(ShooterConstants.conveyor_motor_rps)
        )

    def outtake(self):
        self.conveyor_motor.set_control(
            VelocityVoltage(-ShooterConstants.conveyor_motor_rps)
        )

    @state
    def disable_shooter(self):
        self.upper_roller_motor.stopMotor()
        self.lower_roller_motor.stopMotor()
        self.conveyor_motor.stopMotor()
        self.trigger_motor.stopMotor()
        self.clear_queue()
        self.target_shooter_rps = None
        return True