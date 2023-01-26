package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;


import frc.robot.Constants.OIConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;

    private ArrayList<AprilTag> createTags() {
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
        
        for (Integer id : poses.keySet()) {
            AprilTag tag = new AprilTag(id, poses.get(id));
            out.add(tag);
        }

        return out;
    }

    public Vision() {
        camera = new PhotonCamera(OIConstants.kCameraName);
        ArrayList<AprilTag> tags =  createTags();
        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tags, 596.51, 324);
        
    }
}
