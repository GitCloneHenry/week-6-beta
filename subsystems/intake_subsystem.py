from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, MotorAlignmentValue
from phoenix6.controls import VelocityVoltage, Follower
from configs import IntakeConfigs
from constants import CANConstants, IntakeConstants
from state_system import *

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from subsystems.hopper_subsystem import HopperSubsystem

class IntakeSubsystem(StateSystem):
    left_intake = TalonFX(CANConstants.left_intake_motor)
    right_intake = TalonFX(CANConstants.right_intake_motor)

    intake_follower = Follower(
        CANConstants.right_intake_motor, MotorAlignmentValue.OPPOSED
    )

    intake_toggle = False
    position_override = False
    target_intake_speed = 0.0

    def __init__(self):
        super().__init__()

        self.hopper_subsystem: HopperSubsystem

        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(IntakeConfigs.intake_motor_config)
        self.right_intake.configurator.apply(IntakeConfigs.intake_motor_config)

        self.left_intake.set_control(self.intake_follower)

    def periodic(self):
        super().periodic()

        if not hasattr(self, 'hopper_subsystem'):
            return

        if self.hopper_subsystem.at_target() or self.position_override:
            self.right_intake.set_control(VelocityVoltage(-self.target_intake_speed))
            self.position_override = False

    def toggle_intake(self):
        self.intake_toggle = not self.intake_toggle

        if self.intake_toggle:
            self.target_intake_speed = IntakeConstants.intake_speed
        else:
            self.target_intake_speed = 0

    def toggle_intake_with_override(self):
        self.toggle_intake()
        self.position_override = True

    def outtake(self):
        self.intake_toggle = True
        self.target_intake_speed = -IntakeConstants.intake_speed

    def stop_rollers(self):
        self.intake_toggle = False
        self.target_intake_speed = 0