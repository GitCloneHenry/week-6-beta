from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, MotorAlignmentValue
from phoenix6.controls import VelocityVoltage, Follower
from configs import IntakeConfigs
from constants import CANConstants, IntakeConstants
from state_system import *


class intakeSubsystem(StateSystem):
    left_intake = TalonFX(CANConstants.left_intake_motor)
    right_intake = TalonFX(CANConstants.right_intake_motor)

    intake_follower = Follower(
        CANConstants.right_intake_motor, MotorAlignmentValue.OPPOSED
    )

    intake_toggle = False
    target_intake_speed = 0.0

    def __init__(self):
        super().__init__()
        self.left_intake.setNeutralMode(NeutralModeValue.COAST)
        self.right_intake.setNeutralMode(NeutralModeValue.COAST)

        self.left_intake.configurator.apply(IntakeConfigs.intake_motor_config)
        self.right_intake.configurator.apply(IntakeConfigs.intake_motor_config)

    def periodic(self):
        super().periodic()

        self.left_intake.set_control(VelocityVoltage(self.target_intake_speed))
        self.right_intake.set_control(VelocityVoltage(-self.target_intake_speed))

    @state
    def toggle_intake(self):
        self.intake_toggle = not self.intake_toggle

        if self.intake_toggle:
            self.target_intake_speed = IntakeConstants.intake_speed
        else:
            self.target_intake_speed = 0

        return True

    @state
    def outtake(self):
        self.intake_toggle = True
        self.target_intake_speed = -IntakeConstants.intake_speed

        return True