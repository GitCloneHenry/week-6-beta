from typing import List, Tuple
from constants import VisionConstants

from state_system import *

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Pose3d, Transform3d, Rotation2d, Pose2d
from wpimath.units import degreesToRadians

from photonlibpy import PhotonCamera, PhotonPoseEstimator

from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.targeting import PhotonTrackedTarget

from wpilib import RobotBase

from subsystems.swerve_drive_subsystem import SwerveDriveSubsystem


class VisionCamera:
    def __init__(self, camera_name: str, robot_to_camera: Transform3d):
        self.camera = PhotonCamera(camera_name)
        self.estimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            robot_to_camera,
        )


class VisionSubsystem(StateSystem):
    def __init__(self, drive_subsystem: SwerveDriveSubsystem) -> None:
        self.drive_subsystem: SwerveDriveSubsystem = drive_subsystem

        self.cameras: Tuple[VisionCamera, VisionCamera] = (
            VisionCamera(
                VisionConstants.left_camera_name, VisionConstants.robot_to_left_camera
            ),
            VisionCamera(
                VisionConstants.right_camera_name, VisionConstants.robot_to_right_camera
            ),
        )

        super().__init__()

    def periodic(self):
        super().periodic()

        if not RobotBase.isReal():
            return

        if not hasattr(self, "cameras"):
            return

        try:
            self.april_tag_field_layout = AprilTagFieldLayout.loadField(
                AprilTagField.kDefaultField
            )
        except RuntimeError as e:
            raise e

        current_gyro_rotation: Rotation2d = self.drive_subsystem.get_pose().rotation()

        for camera in self.cameras:
            result: PhotonPipelineResult = camera.camera.getLatestResult()

            if not result.hasTargets():
                continue

            multi_tag_result = result.multitagResult

            target: PhotonTrackedTarget | None = result.getBestTarget()

            if target == None:
                continue

            # distance = target.getBestCameraToTarget().translation().norm()
            # ambiguity = target.getPoseAmbiguity()

            vision_pose = camera.estimator.estimateCoprocMultiTagPose(result)
            
            if vision_pose is None:
                vision_pose = camera.estimator.estimateLowestAmbiguityPose(result)

            if vision_pose is not None:
                robot_pose = vision_pose.estimatedPose.toPose2d()
            else:
                continue

            # if multi_tag_result is not None:
            #     x_stdev = 0.04
            #     y_stdev = 0.04

            #     if distance <= 2.5:
            #         theta_stdev = degreesToRadians(0.1)
            #     else:
            #         theta_stdev = float("inf")
            # else:
            #     if distance <= 2.5 and ambiguity < 0.2:
            #         x_stdev = 0.1 * (distance ** 2)
            #         y_stdev = 0.1 * (distance ** 2)
            #         theta_stdev = 0.2 * (distance ** 2)
            #     else:
            #         x_stdev = 0.2 * (distance ** 2)
            #         y_stdev = 0.2 * (distance ** 2)
            #         theta_stdev = float("inf")
                    
            if RobotBase.isReal():
                self.drive_subsystem.odometry.addVisionMeasurement(
                    robot_pose, vision_pose.timestampSeconds # , (x_stdev, y_stdev, theta_stdev)
                )