from constants import OIConstants
from state_system import *

from commands2 import InstantCommand, ProxyCommand, RunCommand, SequentialCommandGroup
from commands2.button import CommandXboxController

from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem

from wpilib import SendableChooser
from pathplannerlib.auto import AutoBuilder

class RobotContainer(StateSystem):
    hopper_subsystem: HopperSubsystem
    intake_subsystem: IntakeSubsystem
    vision_subsystem: VisionSubsystem 
    drive_subsystem: SwerveDriveSubsystem
    shooter_subsystem: ShooterSubsystem

    def __init__(self) -> None:
        self.hopper_subsystem: HopperSubsystem = HopperSubsystem()
        self.intake_subsystem: IntakeSubsystem = IntakeSubsystem()
        self.vision_subsystem: VisionSubsystem = VisionSubsystem("APTCam")
        self.drive_subsystem: SwerveDriveSubsystem = SwerveDriveSubsystem(self.vision_subsystem)
        self.shooter_subsystem: ShooterSubsystem = ShooterSubsystem(self.drive_subsystem)

        self.hopper_subsystem.intake_subsystem = self.intake_subsystem
        self.intake_subsystem.hopper_subsystem = self.hopper_subsystem

        self.driver_controller = CommandXboxController(OIConstants.driver_controller_port)

        self.sendable_chooser: SendableChooser = AutoBuilder.buildAutoChooser()

    def toggle_intake(self):
        self.hopper_subsystem.toggle_hopper()
        self.intake_subsystem.toggle_intake()

    def outtake(self):
        self.hopper_subsystem.outtake()
        self.intake_subsystem.outtake()
        self.shooter_subsystem.outtake()
    
    def retract(self):
        self.hopper_subsystem.retract()
        self.intake_subsystem.stop_rollers()
        self.shooter_subsystem.disable_shooter()

    def set_controller_bindings(self):
        self.drive_subsystem.setDefaultCommand(
            RunCommand(
                lambda: self.drive_subsystem.default_drive(self.driver_controller, True),
                self.drive_subsystem,
            )
        )

        self.driver_controller.leftBumper().onTrue(
            InstantCommand(lambda: self.drive_subsystem.zero_heading(), self.drive_subsystem)
        )

        self.driver_controller.povRight().onTrue(
            InstantCommand(self.toggle_intake)
        )

        self.driver_controller.a().onTrue(
            InstantCommand(self.outtake)
        )

        self.driver_controller.a().onFalse(
            InstantCommand(self.retract)
        )

        self.driver_controller.b().onTrue(
            InstantCommand(lambda: self.intake_subsystem.toggle_intake_with_override())
        )

        self.driver_controller.rightTrigger().onTrue(
            InstantCommand(lambda: self.shooter_subsystem.queue_states("shoot"))
        )

        self.driver_controller.rightTrigger().onFalse(
            InstantCommand(
                lambda: self.shooter_subsystem.disable_shooter()
            )
        )

    def get_autonomous_command(self):
        return SequentialCommandGroup(
            ProxyCommand(lambda: self.sendable_chooser.getSelected())
        )