from robot_container import RobotContainer

import wpilib


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.robot_container: RobotContainer = RobotContainer()


if __name__ == "__main__":
    wpilib.run(Robot)
