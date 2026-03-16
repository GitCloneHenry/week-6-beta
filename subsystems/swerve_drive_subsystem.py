from math import pi
from typing import Tuple
from numpy import atan2
import wpilib

from constants import CANConstants, DriveConstants, FieldConstants, OIConstants
from phoenix6.hardware import Pigeon2
from subsystems.swerve_module_subsystem import SwerveModuleSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from wpilib import DriverStation
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from commands2.button import CommandXboxController
from commands2.subsystem import Subsystem
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.util import DriveFeedforwards


class SwerveDriveSubsystem(Subsystem):
    front_left: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.front_left_driving_id,
        DriveConstants.front_left_turning_id,
        DriveConstants.front_left_angular_offset_rad,
    )
    front_right: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.front_right_driving_id,
        DriveConstants.front_right_turning_id,
        DriveConstants.front_right_angular_offset_rad,
    )
    back_left: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.back_left_driving_id,
        DriveConstants.back_left_turning_id,
        DriveConstants.back_left_angular_offset_rad,
    )
    back_right: SwerveModuleSubsystem = SwerveModuleSubsystem(
        DriveConstants.back_right_driving_id,
        DriveConstants.back_right_turning_id,
        DriveConstants.back_right_angular_offset_rad,
    )

    gyro: Pigeon2 = Pigeon2(CANConstants.pigeon_id)

    odometry: SwerveDrive4PoseEstimator = SwerveDrive4PoseEstimator(
        DriveConstants.drive_kinematics,
        gyro.getRotation2d(),
        (
            front_left.getPosition(),
            front_right.getPosition(),
            back_left.getPosition(),
            back_right.getPosition(),
        ),
        Pose2d(),
    )

    slow_mode_enabled: bool = False

    x_timer: float | None = None

    def __init__(self, vision_subsystem: VisionSubsystem) -> None:
        self.vision_subsystem = vision_subsystem

        self.theta_pid_controller: ProfiledPIDControllerRadians = (
            ProfiledPIDControllerRadians(
                0.7,
                0.1,
                0.0,
                TrapezoidProfileRadians.Constraints(
                    DriveConstants.max_angular_speed_rps,
                    1.5 * DriveConstants.max_angular_speed_rps,
                ),
            )
        )
        self.theta_pid_controller.enableContinuousInput(-pi, pi)

        try:
            robot_config = RobotConfig.fromGUISettings()

            AutoBuilder.configure(
                self.get_pose,
                self.reset_pose,
                self.get_robot_relative_speeds,
                lambda speeds, feedforwards: self.drive_robot_relative(
                    speeds, feedforwards
                ),
                PPHolonomicDriveController(
                    PIDConstants(13.50, 5.6, 1.9), PIDConstants(9.75, 1.6, 0.6)
                ),
                robot_config,
                lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
                self,
            )
        except Exception as e:
            wpilib.reportError("Error creating configs!")

    def periodic(self) -> None:
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ),
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

    def reset_pose(self, new_pose: Pose2d):
        self.odometry.resetPose(new_pose)

    def apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0
        else:
            return (value - deadband * (1 if value > 0 else -1)) / (1 - deadband)

    def set_module_states(
        self,
        desired_states: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, DriveConstants.max_speed_mps
        )
        self.front_left.setDesiredState(desired_states[0])
        self.front_right.setDesiredState(desired_states[1])
        self.back_left.setDesiredState(desired_states[2])
        self.back_right.setDesiredState(desired_states[3])

    def drive_robot_relative(
        self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards
    ):
        # Convert the desired chassis speeds to individual module states and set the modules to those states. The feedforwards are currently not used, but could be implemented in the future for more accurate control.
        swerve_module_states = DriveConstants.drive_kinematics.toSwerveModuleStates(
            speeds
        )
        self.set_module_states(swerve_module_states)

    def default_drive(
        self, driver_controller: CommandXboxController, field_relative: bool = True
    ):
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
            hub_x, hub_y = FieldConstants.get_hub_pos()

            robot_pose: Pose2d = self.get_pose()
            angle_to_hub: float = -atan2(
                hub_y - robot_pose.Y(),
                hub_x - robot_pose.X(),
            )

            rot = self.theta_pid_controller.calculate(
                robot_pose.rotation().radians(), pi - angle_to_hub
            )
        else:
            rot = -self.apply_deadband(
                driver_controller.getRightX(), OIConstants.drive_deadband
            )

        rot_delivered: float = (
            rot
            * DriveConstants.max_angular_speed_rps
            * (DriveConstants.slow_mode_multiplier if self.slow_mode_enabled else 1.0)
        )

        if x_speed_delivered == 0 and y_speed_delivered == 0 and rot_delivered == 0:
            if self.x_timer is None:
                self.x_timer = wpilib.Timer.getFPGATimestamp()
                self.set_module_states(
                    DriveConstants.drive_kinematics.toSwerveModuleStates(
                        ChassisSpeeds(0, 0, 0)
                    )
                )
            if (
                wpilib.Timer.getFPGATimestamp() - self.x_timer
                > DriveConstants.set_x_duration_s
            ):
                self.set_x()
            return

        self.x_timer = None

        swerve_module_states: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ] = DriveConstants.drive_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered,
                self.gyro.getRotation2d(),
            )
            if field_relative
            else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered)
        )
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, DriveConstants.max_speed_mps
        )
        self.set_module_states(swerve_module_states)

    def set_x(self):
        self.front_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )
        self.front_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.back_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.back_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )

    def get_module_states(
        self,
    ) -> Tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        # Get the current states of all four swerve modules and return them as a list.
        return (
            self.front_left.getState(),
            self.front_right.getState(),
            self.back_left.getState(),
            self.back_right.getState(),
        )

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        # Get the current module states and convert them to robot relative chassis speeds using the kinematics.
        module_states = self.get_module_states()
        robot_relative_speeds = DriveConstants.drive_kinematics.toChassisSpeeds(
            module_states
        )
        return robot_relative_speeds

    def reset_encoders(self):
        # Reset the encoders on all four swerve modules. This should be used when the robot's position on the field is known with certainty, such as at the start of a match or after being picked up by the field staff.
        self.front_left.reset_encoders()
        self.back_left.reset_encoders()
        self.front_right.reset_encoders()
        self.back_right.reset_encoders()

    def zero_heading(self):
        self.gyro.reset()

    def smart_zero_heading(self):
        alliance = DriverStation.getAlliance()

        if alliance != None:
            self.gyro.set_yaw(
                self.get_pose().rotation().degrees()
                + 180.0 * (alliance == DriverStation.Alliance.kRed)
            )
        else:
            wpilib.reportError("Couldn't get alliance!")
