package com.team2914.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2914.robot.Constants.OIConstants;
import com.team2914.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private static Vision instance = null;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final NetworkTableInstance ntInst;
    private final NetworkTable roboflowTable;
    private final StringSubscriber clawTargetName;

    private Vision() {
        camera = new PhotonCamera(OIConstants.PHOTON_CAMERA_NAME);
        AprilTagFieldLayout fieldLayout = null;
        
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load AprilTagFieldLayout. Error: " + e);
        }

        poseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                                                camera, 
                                                VisionConstants.ROBOT_TO_CAM);

        ntInst = NetworkTableInstance.getDefault();
        roboflowTable = ntInst.getTable("/roboflow/");
        StringTopic clawTargetNameTopic = roboflowTable.getStringTopic("clawTargetName");
        clawTargetName = clawTargetNameTopic.subscribe("none");
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Claw target", clawTargetName.get());
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d previousEstimatedPose) {
        poseEstimator.setReferencePose(previousEstimatedPose);
        return poseEstimator.update();

    }

    public PhotonTrackedTarget getBestTarget() {
        return camera.getLatestResult().getBestTarget();
    }

    public void closeRoboflow() {
        ntInst.close();
    }
}