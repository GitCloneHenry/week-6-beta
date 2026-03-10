from math import cos, pi, sin
from typing import List
from numpy import atan2
import wpilib

from constants import (
    CANConstants,
    DriveConstants,
    FieldConstants,
    OIConstants,
    AutoConstants,
)
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.util import DriveFeedforwards
from phoenix6.hardware import Pigeon2
from subsystems.swerve_module_subsystem import SwerveModuleSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from wpilib import DriverStation, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from commands2.button import CommandXboxController
from commands2.subsystem import Subsystem
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

class SwerveDriveSubsystem(Subsystem):
    front_left: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.front_left_driving_id,
        DriveConstants.front_left_turning_id,
        DriveConstants.front_left_angular_offset_rad
    )
    front_right: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.front_right_driving_id,
        DriveConstants.front_right_turning_id,
        DriveConstants.front_right_angular_offset_rad
    )
    back_left: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.back_left_driving_id,
        DriveConstants.back_left_turning_id,
        DriveConstants.back_left_angular_offset_rad
    )
    back_right: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.back_right_driving_id,
        DriveConstants.back_right_turning_id,
        DriveConstants.back_right_angular_offset_rad
    )

    gyro: Pigeon2 = Pigeon2(CANConstants.pigeon_id)

    odometry: SwerveDrive4PoseEstimator = SwerveDrive4PoseEstimator(
        DriveConstants.drive_kinematics,
        gyro.getRotation2d(),
        (front_left.getPosition(), front_right.getPosition(), back_left.getPosition(), back_right.getPosition()),
        Pose2d()
    ) 

    slow_mode_enabled: bool = False

    def __init__(self, vision_subsystem: VisionSubsystem) -> None:
        self.vision_subsystem = vision_subsystem

        self.theta_pid_controller: ProfiledPIDControllerRadians = ProfiledPIDControllerRadians(
            0.7, 0.1, 0.0, TrapezoidProfileRadians.Constraints(DriveConstants.max_angular_speed_rps, 1.5 * DriveConstants.max_angular_speed_rps))
        self.theta_pid_controller.enableContinuousInput(-pi, pi)


    def periodic(self) -> None:
        self.odometry.update(
            self.gyro.getRotation2d(),
            (self.front_left.getPosition(), self.front_right.getPosition(), self.back_left.getPosition(), self.back_right.getPosition())
        )

        if self.vision_subsystem.robot_pose != None:
            robot_pose = self.vision_subsystem.robot_pose
            self.odometry.addVisionMeasurement(
                robot_pose.estimatedPose.toPose2d(), robot_pose.timestampSeconds
            )

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()
    
    def reset_odometry(self, pose: Pose2d) -> None:
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ),
            pose,
        )

    def apply_deadband(self, value: float, deadband: float) -> float:
        """Applies a deadband to the given value. If the absolute value of the input is less than the deadband, returns 0. Otherwise, scales the input so that it starts from 0 at the edge of the deadband and reaches 1 at the maximum input."""
        if abs(value) < deadband:
            return 0.0
        else:
            return (value - deadband * (1 if value > 0 else -1)) / (1 - deadband)


    def default_drive(self, driver_controller: CommandXboxController, field_relative: bool = True):
        if DriverStation.isDisabled():
            return
        
        x_speed = -self.apply_deadband(
            driver_controller.getLeftY(), OIConstants.drive_deadband
        )
        y_speed = -self.apply_deadband(
            driver_controller.getLeftX(), OIConstants.drive_deadband
        )
        
        x_speed_delivered: float = (
            x_speed
            * DriveConstants.max_speed_mps
            * (DriveConstants.slow_mode_multiplier if self.slow_mode_enabled else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )
        y_speed_delivered: float = (
            y_speed
            * DriveConstants.max_speed_mps
            * (DriveConstants.slow_mode_multiplier if self.slow_mode_enabled else 1.0)
            * (-1.0 if DriveConstants.gyro_reversed else 1.0)
        )

        if driver_controller.leftTrigger().getAsBoolean():
            robot_pose: Pose2d = self.get_pose()
            angle_to_hub: float = -atan2(
                FieldConstants.hub_y - robot_pose.Y(), FieldConstants.hub_x - robot_pose.X())
            
            rot = self.theta_pid_controller.calculate(robot_pose.rotation().radians(), pi - angle_to_hub)
        else:
            rot = -self.apply_deadband(
                driver_controller.getRightX(), OIConstants.drive_deadband
            )
        
        rot_delivered: float = (
            rot
            * DriveConstants.max_angular_speed_rps
            * (DriveConstants.slow_mode_multiplier if self.slow_mode_enabled else 1.0)
        )