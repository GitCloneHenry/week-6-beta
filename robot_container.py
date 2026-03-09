from state_system import *

from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem

class RobotContainer(StateSystem):
    def __init__(self) -> None:
        self.hopper_subsystem: HopperSubsystem = HopperSubsystem()
        self.intake_subsystem: IntakeSubsystem = IntakeSubsystem()
        self.shooter_subsystem: ShooterSubsystem = ShooterSubsystem()
        self.drive_subsystem: SwerveDriveSubsystem = SwerveDriveSubsystem()