from robot_container import RobotContainer
from commands2 import Command, CommandScheduler
from wpilib import RobotController
from wpilib import SmartDashboard, Field2d

import wpilib


class Robot(wpilib.TimedRobot):
    robot_container = RobotContainer()
    field = Field2d()
    autonomous_command: Command | None = None

    def robotInit(self):
        RobotController.setBrownoutVoltage(7.0)

        SmartDashboard.putData("Field", self.field)
        SmartDashboard.putData("Auto Chooser", self.robot_container.sendable_chooser)

    def robotPeriodic(self):
        self.field.setRobotPose(self.robot_container.drive_subsystem.get_pose())

        CommandScheduler.getInstance().run()

    def teleopInit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

        self.robot_container.shooter_subsystem.disable_shooter()
        self.robot_container.drive_subsystem.smart_zero_heading()

    def autonomousInit(self):
        self.autonomous_command = self.robot_container.get_autonomous_command()

        if self.autonomous_command:
            self.autonomous_command.schedule()


if __name__ == "__main__":
    wpilib.run(Robot)