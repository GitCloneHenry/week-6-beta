from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import NeutralModeValue, MotorAlignmentValue
from phoenix6.controls import MotionMagicVoltage, Follower
from configs import HopperConfigs
from constants import CANConstants, HopperConstants
from state_system import *
from time import sleep
from commands2.button import CommandXboxController


class HopperSubsystem(StateSystem):
    left_intake = TalonFXS(CANConstants.left_hopper_motor)
    right_intake = TalonFXS(CANConstants.right_hopper_motor)

    intake_follower = Follower(
        CANConstants.right_intake_motor, MotorAlignmentValue.OPPOSED
    )

    hopper_toggle = False
    target_hopper_position = 0.0

    def __init__(self):
        super().__init__()
        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(HopperConfigs.hopper_motor_config)
        self.right_intake.configurator.apply(HopperConfigs.hopper_motor_config)

    def periodic(self):
        super().periodic()

        self.left_intake.set_control(MotionMagicVoltage(self.target_hopper_position))
        self.right_intake.set_control(MotionMagicVoltage(-self.target_hopper_position))

    @state
    def toggle_hopper(self):
        self.hopper_toggle = not self.hopper_toggle

        if self.hopper_toggle:
            self.target_hopper_position = HopperConstants.extended_position
        else:
            self.target_hopper_position = HopperConstants.retracted_position

        return True

    @state
    def outtake(self):
        self.hopper_toggle = True
        self.target_hopper_position = HopperConstants.extended_position

        return True