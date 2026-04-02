from math import pi
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
from typing import Dict, Tuple


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
    retracted_position: float = 0.5

    minimum_acceptable_closed_loop_error: float = 1.0


class IntakeConstants:
    intake_speed: float = KrakenX44Constants.free_speed_rpm * 0.9
    outtake_speed: float = -KrakenX44Constants.free_speed_rpm * 0.9


class ShooterConstants:
    advancement_motor_rps: float = -90
    conveyor_motor_rps: float = -90

    minimum_acceptable_closed_loop_error: float = 1.0

    backspin_multiplier: float = 1.25

    # Values calculated using 5962's ProjectileSimulator (https://github.com/eeveemara/frc-fire-control)
    rpm_table: Dict[float, float] = {
        0.55: 6000,
        0.60: 3750,
        0.65: 2344,
        0.70: 1922,
        0.75: 1781,
        0.80: 1711,
        0.85: 1641,
        0.90: 1605,
        0.95: 1570,
        1.00: 1570,
        1.05: 1570,
        1.10: 1570,
        1.15: 1570,
        1.20: 1570,
        1.25: 1570,
        1.30: 1570,
        1.35: 1588,
        1.40: 1588,
        1.45: 1605,
        1.50: 1623,
        1.55: 1623,
        1.60: 1641,
        1.65: 1658,
        1.70: 1667,
        1.75: 1676,
        1.80: 1693,
        1.85: 1711,
        1.90: 1720,
        1.95: 1737,
        2.00: 1746,
        2.05: 1764,
        2.10: 1781,
        2.15: 1790,
        2.20: 1808,
        2.25: 1816,
        2.30: 1834,
        2.35: 1852,
        2.40: 1860,
        2.45: 1878,
        2.50: 1896,
        2.55: 1904,
        2.60: 1922,
        2.65: 1939,
        2.70: 1948,
        2.75: 1966,
        2.80: 1979,
        2.85: 1992,
        2.90: 2010,
        2.95: 2019,
        3.00: 2036,
        3.05: 2049,
        3.10: 2063,
        3.15: 2080,
        3.20: 2093,
        3.25: 2106,
        3.30: 2120,
        3.35: 2133,
        3.40: 2150,
        3.45: 2159,
        3.50: 2177,
        3.55: 2190,
        3.60: 2203,
        3.65: 2216,
        3.70: 2229,
        3.75: 2247,
        3.80: 2256,
        3.85: 2273,
        3.90: 2282,
        3.95: 2300,
        4.00: 2313,
        4.05: 2326,
        4.10: 2339,
        4.15: 2353,
        4.20: 2366,
        4.25: 2379,
        4.30: 2392,
        4.35: 2405,
        4.40: 2418,
        4.45: 2432,
        4.50: 2445,
        4.55: 2458,
        4.60: 2471,
        4.65: 2484,
        4.70: 2498,
        4.75: 2511,
        4.80: 2524,
        4.85: 2537,
        4.90: 2546,
        4.95: 2559,
        5.00: 2572,
    }

    @staticmethod
    def get_shooter_rpm(distance: float) -> float:
        if distance <= 0.55:
            return ShooterConstants.rpm_table[0.55] / 60.0
        elif distance >= 5.0:
            return ShooterConstants.rpm_table[5.0] / 60.0
        else:
            lower_bound = max(
                key for key in ShooterConstants.rpm_table.keys() if key <= distance
            )
            upper_bound = min(
                key for key in ShooterConstants.rpm_table.keys() if key >= distance
            )

            lower_rpm = ShooterConstants.rpm_table[lower_bound]
            upper_rpm = ShooterConstants.rpm_table[upper_bound]

            rpm = lower_rpm + (upper_rpm - lower_rpm) * (
                (distance - lower_bound) / (upper_bound - lower_bound)
            )
            return rpm / 60.0


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

    gyro_reversed: bool = True

    set_x_duration_s: float = 0.25


class ModuleConstants:
    driving_motor_pinion_teeth: int = 13

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
    left_camera_name = "APTCam1"
    right_camera_name = "APTCam2"

    robot_to_left_camera = Transform3d(
        Translation3d(
            inchesToMeters(-12.5102), inchesToMeters(-0.9959), inchesToMeters(16.9634)
        ),
        Rotation3d(pi / 6, pi - 14.551811 * pi / 180 - 7 * pi / 180, 0),
    )
    robot_to_right_camera = Transform3d(
        Translation3d(
            inchesToMeters(-12.5102), inchesToMeters(0.9959), inchesToMeters(16.9634)
        ),
        Rotation3d(pi / 6, pi + 14.551811 * pi / 180 - 7 * pi / 180, 0),
    )


class FieldConstants:
    # red_hub_pose = Pose2d(11.834, 4.035, Rotation2d(0))
    red_hub_pose = Pose2d(11.920, 4.035, Rotation2d(0))
    # blue_hub_pose = Pose2d(4.706, 4.035, Rotation2d(0))
    blue_hub_pose = Pose2d(4.630, 4.035, Rotation2d(0))

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


class SOTMConstants:
    launcher_offset_x: float = 0.2
    launcher_offset_y: float = 0.0

    min_scoring_distance: float = 0.5
    max_scoring_distance: float = 5.0

    max_iterations: int = 25
    convergance_tolerance: float = 0.001
    tof_min: float = 0.05
    tof_max: float = 5.0

    min_sotm_speed: float = 0.1

    max_sotm_speed: float = 3.0

    phase_delay_ms: float = 30.0
    mech_latency_ms: float = 20.0

    sotm_drag_coeff: float = 0.24

    convergence: float = 1.0
    velocity_stability: float = 0.8
    vision_confidence: float = 1.2
    heading_accuracy: float = 1.5
    distance_in_range: float = 0.5
    heading_max_error_rad = 15 * pi / 180

    heading_speed_scalar: float = 1.0

    heading_reference_distance: float = 2.5

    max_tilt_deg: float = 5.0

    shooter_angle_offset_rad: float = 0.0