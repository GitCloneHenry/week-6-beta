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

    minimum_acceptable_closed_loop_error: float = 2.0

    current_threshold: float = 17.0


class IntakeConstants:
    intake_speed: float = KrakenX44Constants.free_speed_rpm * 0.6
    outtake_speed: float = -KrakenX44Constants.free_speed_rpm * 0.6


class ShooterConstants:
    advancement_motor_rps: float = -90
    conveyor_motor_rps: float = -90

    minimum_acceptable_closed_loop_error: float = 1.0

    backspin_multiplier: float = 1.2
    backspin_correction_multiplier: float = 2 - backspin_multiplier

    topspin_multiplier: float = 1.75
    topspin_correction_multiplier: float = 1.25

    # Values calculated using 5962's ProjectileSimulator (https://github.com/eeveemara/frc-fire-control)
    rpm_table: Dict[float, float] = {
        0.55: 6000,
        0.60: 3750,
        0.65: 2625,
        0.70: 2203,
        0.75: 2063,
        0.80: 1922,
        0.85: 1852,
        0.90: 1816,
        0.95: 1781,
        1.00: 1781,
        1.05: 1746,
        1.10: 1746,
        1.15: 1746,
        1.20: 1764,
        1.25: 1764,
        1.30: 1781,
        1.35: 1781,
        1.40: 1799,
        1.45: 1799,
        1.50: 1816,
        1.55: 1834,
        1.60: 1852,
        1.65: 1860,
        1.70: 1869,
        1.75: 1887,
        1.80: 1904,
        1.85: 1922,
        1.90: 1939,
        1.95: 1948,
        2.00: 1966,
        2.05: 1983,
        2.10: 2001,
        2.15: 2019,
        2.20: 2027,
        2.25: 2045,
        2.30: 2063,
        2.35: 2080,
        2.40: 2098,
        2.45: 2115,
        2.50: 2133,
        2.55: 2142,
        2.60: 2159,
        2.65: 2177,
        2.70: 2194,
        2.75: 2212,
        2.80: 2229,
        2.85: 2243,
        2.90: 2256,
        2.95: 2273,
        3.00: 2291,
        3.05: 2309,
        3.10: 2322,
        3.15: 2335,
        3.20: 2353,
        3.25: 2370,
        3.30: 2388,
        3.35: 2401,
        3.40: 2414,
        3.45: 2432,
        3.50: 2449,
        3.55: 2462,
        3.60: 2476,
        3.65: 2493,
        3.70: 2511,
        3.75: 2524,
        3.80: 2537,
        3.85: 2555,
        3.90: 2572,
        3.95: 2585,
        4.00: 2599,
        4.05: 2616,
        4.10: 2634,
        4.15: 2647,
        4.20: 2660,
        4.25: 2678,
        4.30: 2691,
        4.35: 2704,
        4.40: 2722,
        4.45: 2735,
        4.50: 2748,
        4.55: 2766,
        4.60: 2779,
        4.65: 2792,
        4.70: 2810,
        4.75: 2823,
        4.80: 2836,
        4.85: 2854,
        4.90: 2867,
        4.95: 2880,
        5.00: 2897,
        5.05: 2911,
        5.10: 2924,
        5.15: 2937,
        5.20: 2950,
        5.25: 2968,
        5.30: 2981,
        5.35: 2994,
        5.40: 3012,
        5.45: 3025,
        5.50: 3038,
        5.55: 3051,
        5.60: 3064,
        5.65: 3078,
        5.70: 3095,
        5.75: 3108,
        5.80: 3122,
        5.85: 3135,
        5.90: 3148,
        5.95: 3161,
        6.00: 3179,
        6.05: 3192,
        6.10: 3205,
        6.15: 3218,
        6.20: 3231,
        6.25: 3245,
        6.30: 3258,
        6.35: 3271,
        6.40: 3289,
        6.45: 3302,
        6.50: 3315,
        6.55: 3328,
        6.60: 3341,
        6.65: 3354,
        6.70: 3368,
        6.75: 3381,
        6.80: 3394,
        6.85: 3407,
        6.90: 3420,
        6.95: 3434,
        7.00: 3447,
        7.05: 3460,
        7.10: 3478,
        7.15: 3491,
        7.20: 3504,
        7.25: 3517,
        7.30: 3530,
        7.35: 3543,
        7.40: 3557,
        7.45: 3570,
        7.50: 3583,
        7.55: 3596,
        7.60: 3609,
        7.65: 3623,
        7.70: 3636,
        7.75: 3649,
        7.80: 3662,
        7.85: 3675,
        7.90: 3688,
        7.95: 3702,
        8.00: 3715,
        8.05: 3728,
        8.10: 3741,
        8.15: 3754,
        8.20: 3768,
        8.25: 3781,
        8.30: 3794,
        8.35: 3807,
        8.40: 3820,
        8.45: 3833,
        8.50: 3844,
        8.55: 3858,
        8.60: 3871,
        8.65: 3882,
        8.70: 3895,
        8.75: 3908,
        8.80: 3921,
        8.85: 3935,
        8.90: 3948,
        8.95: 3961,
        9.00: 3974,
        9.05: 3987,
        9.10: 4000,
        9.15: 4014,
        9.20: 4027,
        9.25: 4040,
        9.30: 4053,
        9.35: 4066,
        9.40: 4080,
        9.45: 4091,
        9.50: 4104,
        9.55: 4115,
        9.60: 4128,
        9.65: 4141,
        9.70: 4154,
        9.75: 4167,
        9.80: 4181,
        9.85: 4194,
        9.90: 4207,
        9.95: 4220,
        10.00: 4233,
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
            inchesToMeters(-12.9004),
            inchesToMeters(-0.995855),
            inchesToMeters(14.492716),
        ),
        Rotation3d(pi / 6, pi - 14.551811 * pi / 180, 0),
    )
    robot_to_right_camera = Transform3d(
        Translation3d(
            inchesToMeters(-12.9004),
            inchesToMeters(0.995855),
            inchesToMeters(14.492716),
        ),
        Rotation3d(pi / 6, pi + 14.551811 * pi / 180, 0),
    )


class FieldConstants:
    # red_hub_pose = Pose2d(11.834, 4.035, Rotation2d(0))
    red_hub_pose = Pose2d(11.920, 4.035, Rotation2d(0))
    # blue_hub_pose = Pose2d(4.706, 4.035, Rotation2d(0))
    blue_hub_pose = Pose2d(4.530, 4.035, Rotation2d(0))

    blue_alliance_end: float = 5.150
    red_alliance_start: float = 11.400

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
