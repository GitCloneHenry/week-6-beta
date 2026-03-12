from phoenix6.hardware import TalonFXS
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicVoltage
from configs import HopperConfigs
from constants import CANConstants, HopperConstants
from state_system import *

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from subsystems.intake_subsystem import IntakeSubsystem


class HopperSubsystem(StateSystem):
    left_hopper_motor = TalonFXS(CANConstants.left_hopper_motor)
    right_hopper_motor = TalonFXS(CANConstants.right_hopper_motor)

    hopper_toggle = False
    target_hopper_position = 0.0

    def __init__(self):
        super().__init__()

        self.intake_subsystem: IntakeSubsystem

        self.left_hopper_motor.setNeutralMode(NeutralModeValue.COAST)
        self.right_hopper_motor.setNeutralMode(NeutralModeValue.COAST)

        self.left_hopper_motor.configurator.apply(HopperConfigs.hopper_motor_config)
        self.right_hopper_motor.configurator.apply(HopperConfigs.hopper_motor_config)

    def periodic(self):
        super().periodic()

        self.left_hopper_motor.set_control(
            MotionMagicVoltage(self.target_hopper_position)
        )
        self.right_hopper_motor.set_control(
            MotionMagicVoltage(-self.target_hopper_position)
        )

    def at_target(self) -> bool:
        left_error: float = abs(
            self.left_hopper_motor.get_position().value_as_double
            - self.target_hopper_position
        )
        right_error: float = abs(
            self.right_hopper_motor.get_position().value_as_double
            - (-self.target_hopper_position)
        )
        return (
            left_error < HopperConstants.minimum_acceptable_closed_loop_error
            and right_error < HopperConstants.minimum_acceptable_closed_loop_error
        )

    def toggle_hopper(self):
        self.hopper_toggle = not self.hopper_toggle
        self.intake_subsystem.intake_toggle = self.hopper_toggle

        if self.hopper_toggle:
            self.target_hopper_position = HopperConstants.extended_position
        else:
            self.target_hopper_position = HopperConstants.retracted_position

    def outtake(self):
        self.hopper_toggle = True
        self.target_hopper_position = HopperConstants.extended_position

    def retract(self):
        self.hopper_toggle = False
        self.target_hopper_position = HopperConstants.retracted_position
