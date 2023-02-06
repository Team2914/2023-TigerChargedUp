package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;

public class PhotonAprilTags {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonAprilTags() {
        camera = new PhotonCamera(OIConstants.kCameraName);
        AprilTagFieldLayout fieldLayout = null;
        
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load AprilTagFieldLayout. Error: " + e);
        }

        poseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                                                camera, 
                                                VisionConstants.kRobotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d previousEstimatedPose) {
        poseEstimator.setReferencePose(previousEstimatedPose);
        return poseEstimator.update();
    }
}