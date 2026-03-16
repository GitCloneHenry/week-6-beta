from math import exp, pi
from wpimath.geometry import (
    Translation2d,
    Translation3d,
    Transform3d,
    Rotation2d,
    Rotation3d,
    Pose2d,
)
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters
from wpilib import DriverStation
from typing import Callable, Tuple


class NeoMotorConstants:
    free_speed_rpm: float = 5676


class KrakenX60Constants:
    free_speed_rpm: float = 6136


class KrakenX44Constants:
    free_speed_rpm: float = 7644


class CANConstants:
    pigeon_id: int = 0
    right_hopper_motor: int = 1
    left_hopper_motor: int = 2
    upper_roller_motor: int = 3
    lower_roller_motor: int = 4
    conveyor_motor: int = 5
    trigger_motor: int = 6
    right_intake_motor: int = 7
    left_intake_motor: int = 8


class HopperConstants:
    extended_position: float = 15.5
    retracted_position: float = 0.0

    minimum_acceptable_closed_loop_error: float = 1.0


class IntakeConstants:
    intake_speed: float = KrakenX44Constants.free_speed_rpm * 0.9
    outtake_speed: float = -KrakenX44Constants.free_speed_rpm * 0.9


class ShooterConstants:
    get_shooter_rpm: Callable[[float], float] = (
        lambda dist: 0.345585037558 * exp(1.09866543407 * dist) + 26.1542760624
    )
    advancement_motor_rps: float = -90
    conveyor_motor_rps: float = -90

    minimum_acceptable_closed_loop_error: float = 1.0


class DriveConstants:
    max_speed_mps: float = 4.8
    max_angular_speed_rps: float = 2 * pi
    slow_mode_multiplier: float = 0.3

    track_width_m: float = 0.5969
    wheel_base_m: float = 0.5969
    drive_kinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(
        Translation2d(wheel_base_m / 2, track_width_m / 2),
        Translation2d(wheel_base_m / 2, -track_width_m / 2),
        Translation2d(-wheel_base_m / 2, track_width_m / 2),
        Translation2d(-wheel_base_m / 2, -track_width_m / 2),
    )

    front_left_angular_offset_rad: float = -pi / 2
    front_right_angular_offset_rad: float = 0.0
    back_left_angular_offset_rad: float = pi
    back_right_angular_offset_rad: float = pi / 2

    front_left_driving_id: int = 2
    front_right_driving_id: int = 4
    back_left_driving_id: int = 6
    back_right_driving_id: int = 8

    front_left_turning_id: int = 1
    front_right_turning_id: int = 3
    back_left_turning_id: int = 5
    back_right_turning_id: int = 7

    gyro_reversed: bool = False

    set_x_duration_s: float = 0.25


class ModuleConstants:
    driving_motor_pinion_teeth: int = 12

    driving_motor_free_speed_rps = NeoMotorConstants.free_speed_rpm / 60.0
    wheel_diameter_m = 0.0762
    wheel_circumference_m = wheel_diameter_m * pi

    driving_motor_reduction = (45.0 * 22.0) / (float(driving_motor_pinion_teeth) * 15)
    drive_wheel_free_speed_rps = (
        driving_motor_free_speed_rps * wheel_circumference_m
    ) / driving_motor_reduction


class OIConstants:
    driver_controller_port: int = 0

    drive_deadband: float = 0.05


class VisionConstants:
    robot_to_camera = Transform3d(
        Translation3d(inchesToMeters(-12.5), 0, inchesToMeters(15.0)),
        Rotation3d(pi / 6, pi, 0),
    )


class FieldConstants:
    red_hub_pose = Pose2d(11.834, 4.035, Rotation2d(0))
    blue_hub_pose = Pose2d(4.706, 4.035, Rotation2d(0))

    @staticmethod
    def get_hub_dist(pose: Pose2d) -> float:
        hub_x, hub_y = (
            (FieldConstants.red_hub_pose.X(), FieldConstants.red_hub_pose.Y())
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else (FieldConstants.blue_hub_pose.X(), FieldConstants.blue_hub_pose.Y())
        )

        return ((pose.X() - hub_x) ** 2 + (pose.Y() - hub_y) ** 2) ** 0.5

    @staticmethod
    def get_hub_pos() -> Tuple[float, float]:
        hub_x, hub_y = (
            (FieldConstants.red_hub_pose.X(), FieldConstants.red_hub_pose.Y())
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else (FieldConstants.blue_hub_pose.X(), FieldConstants.blue_hub_pose.Y())
        )
        return hub_x, hub_y