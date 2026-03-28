from typing import List, Tuple
from constants import VisionConstants

from state_system import *

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Pose3d, Transform3d

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

        for camera in self.cameras:
            camera_results: List[PhotonPipelineResult] = (
                camera.camera.getAllUnreadResults()
            )

            for result in camera_results:
                if not result.hasTargets():
                    continue

                best_target: PhotonTrackedTarget | None = result.getBestTarget()

                if best_target == None:
                    continue

                potential_tag_pose: Pose3d | None = (
                    self.april_tag_field_layout.getTagPose(best_target.getFiducialId())
                )

                if not potential_tag_pose == None:
                    self.best_april_tag_pose = potential_tag_pose.toPose2d()

                robot_pose = camera.estimator.estimateCoprocMultiTagPose(result)

                if robot_pose is None:
                    robot_pose = camera.estimator.estimateLowestAmbiguityPose(result)

                if robot_pose:
                    self.drive_subsystem.odometry.addVisionMeasurement(
                        robot_pose.estimatedPose.toPose2d(), robot_pose.timestampSeconds
                    )
                    