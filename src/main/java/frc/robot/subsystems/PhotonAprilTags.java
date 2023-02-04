package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.apriltag.AprilTag;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;

public class PhotonAprilTags {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private ArrayList<AprilTag> createTags() {
        // TODO: put all of these into JSON
        HashMap<Integer, Pose3d> poses = new HashMap<Integer, Pose3d>();
        ArrayList<AprilTag> out = new ArrayList<AprilTag>();
        poses.put(1, new Pose3d(
            new Translation3d(610.77, 42.19, 18.22),
            new Rotation3d(0, 0, 180)
        ));
        poses.put(2, new Pose3d(
            new Translation3d(610.77, 108.19, 18.22),
            new Rotation3d(0, 0, 180)
        ));
        poses.put(3, new Pose3d(
            new Translation3d(610.77, 174.19, 18.22),
            new Rotation3d(0, 0, 180)
        ));
        poses.put(4, new Pose3d(
            new Translation3d(636.96, 265.74, 27.38),
            new Rotation3d(0,0,180)
        ));
        poses.put(5, new Pose3d(
            new Translation3d(14.25, 265.74, 27.38),
            new Rotation3d(0,0,0)
        ));
        poses.put(6, new Pose3d(
            new Translation3d(40.45, 174.19, 18.22),
            new Rotation3d(0,0,0)
        ));
        poses.put(7, new Pose3d(
            new Translation3d(40.45,  108.19, 18.22),
            new Rotation3d(0,0,0)
        ));
        poses.put(8, new Pose3d(
            new Translation3d(40.45, 174.19, 18.22),
            new Rotation3d(0,0,0)
        ));
        
        for (Integer id : poses.keySet()) {
            AprilTag tag = new AprilTag(id, poses.get(id));
            out.add(tag);
        }

        return out;
    }

    public PhotonAprilTags() {
        camera = new PhotonCamera(OIConstants.kCameraName);
        ArrayList<AprilTag> tags =  createTags();
        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tags, 648, 324);
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