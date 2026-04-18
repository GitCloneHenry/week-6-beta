from robot_container import RobotContainer
from commands2 import Command, CommandScheduler
from wpilib import SmartDashboard, Field2d

import wpilib

from wpilib import DataLogManager, DriverStation

class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()
    field = Field2d()
    autonomous_command: Command | None = None

    def robotInit(self):
        SmartDashboard.putData("Field", self.field)
        SmartDashboard.putData("Auto Chooser", self.robot_container.sendable_chooser)

        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

    def robotPeriodic(self):
        self.field.setRobotPose(self.robot_container.drive_subsystem.get_pose())

        CommandScheduler.getInstance().run()

    def teleopInit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

        self.robot_container.shooter_subsystem.disable_shooter()
        self.robot_container.drive_subsystem.smart_zero_heading()
        self.robot_container.intake_subsystem.force_run()

    def autonomousInit(self):
        self.autonomous_command = self.robot_container.get_autonomous_command()

        if self.autonomous_command:
            self.autonomous_command.schedule()


if __name__ == "__main__":
    wpilib.run(Robot)
