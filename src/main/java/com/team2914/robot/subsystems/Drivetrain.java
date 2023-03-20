// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team2914.lib.gyro.BNO055;
import com.team2914.lib.gyro.BNO055.opmode_t;
import com.team2914.lib.gyro.BNO055.vector_type_t;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team2914.robot.Constants.AutoConstants;
import com.team2914.robot.Constants.DriveConstants;
import com.team2914.robot.utils.MathUtil;
import com.team2914.robot.utils.MiscUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = null;
  // Create MAXSwerveModules
  private final MAXSwerveModule moduleFrontLeft = new MAXSwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_CAN_ID,
      DriveConstants.FRONT_LEFT_TURN_CAN_ID,
      DriveConstants.FRONT_LEFT_ANGULAR_OFFSET);

  private final MAXSwerveModule moduleFrontRight = new MAXSwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_CAN_ID,
      DriveConstants.FRONT_RIGHT_TURN_CAN_ID,
      DriveConstants.FRONT_RIGHT_ANGULAR_OFFSET);

  private final MAXSwerveModule moduleRearLeft = new MAXSwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_CAN_ID,
      DriveConstants.REAR_LEFT_TURN_CAN_ID,
      DriveConstants.BACK_LEFT_ANGULAR_OFFSET);

  private final MAXSwerveModule moduleRearRight = new MAXSwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_CAN_ID,
      DriveConstants.REAR_RIGHT_TURN_CAN_ID,
      DriveConstants.BACK_RIGHT_ANGULAR_OFFSET);

  //private final AHRS gyro;
  private final BNO055 gyro;
  private final Vision vision;
  private Field2d field = new Field2d();
  SwerveDrivePoseEstimator poseEstimator;
  boolean isFieldRelative = false;
  FieldLocation currentLocation;

  /** Creates a new DriveSubsystem. */
  private Drivetrain() {
    //gyro = new AHRS(I2C.Port.kMXP);
    gyro = BNO055.getInstance(opmode_t.OPERATION_MODE_IMUPLUS, vector_type_t.VECTOR_EULER);
    vision = Vision.getInstance();

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
          moduleFrontLeft.getPosition(),
          moduleFrontRight.getPosition(),
          moduleRearLeft.getPosition(),
          moduleRearRight.getPosition()
      },
      new Pose2d());

    SmartDashboard.putData("Field", field);
    currentLocation = FieldLocation.OPEN;
  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gyro calibrated?", gyro.isCalibrated());
    SmartDashboard.putNumber("Heading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("target x", 2);
    SmartDashboard.putBoolean("Field relative", isFieldRelative);
    SmartDashboard.putString("Current location", currentLocation.name());
    SmartDashboard.putNumber("Robot pitch", gyro.getVector()[2]);

    poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            moduleFrontLeft.getPosition(),
            moduleFrontRight.getPosition(),
            moduleRearLeft.getPosition(),
            moduleRearRight.getPosition()
        });

    field.setRobotPose(poseEstimator.getEstimatedPosition()); 

    Optional<EstimatedRobotPose> camResult = vision.getEstimatedRobotPose(poseEstimator.getEstimatedPosition());

    if (camResult.isPresent()) {
      EstimatedRobotPose camPose = camResult.get();
      double distance = 
        new Translation2d(
            camPose.estimatedPose.getTranslation().getX(), 
            camPose.estimatedPose.getTranslation().getY())
        .getDistance(poseEstimator.getEstimatedPosition().getTranslation());
      poseEstimator.addVisionMeasurement(
        camPose.estimatedPose.toPose2d(), 
        camPose.timestampSeconds, 
        VecBuilder.fill(distance / 2, distance / 2, 100));
    }
    
    currentLocation = FieldLocation.OPEN;
    Translation2d translation = poseEstimator.getEstimatedPosition().getTranslation();
    if (MiscUtil.isInsideBoundingBox(translation, AutoConstants.BLU_GRID1_BB_TOP_LEFT, AutoConstants.BLU_GRID1_BB_BTM_RITE)) {
      currentLocation = FieldLocation.BLU_GRID_1;
    }
    if (MiscUtil.isInsideBoundingBox(translation, AutoConstants.BLU_GRID2_BB_TOP_LEFT, AutoConstants.BLU_GRID2_BB_BTM_RITE)) {
      currentLocation = FieldLocation.BLU_GRID_2;
    }
    if (MiscUtil.isInsideBoundingBox(translation, AutoConstants.BLU_GRID3_BB_TOP_LEFT, AutoConstants.BLU_GRID3_BB_BTM_RITE)) {
      currentLocation = FieldLocation.BLU_GRID_3;
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }


  public void drive(double xSpeed, double ySpeed, double rot) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    rot *= DriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(isFieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().rotateBy(Rotation2d.fromDegrees(180))) :
      new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    moduleFrontLeft.setDesiredState(swerveModuleStates[0]);
    moduleFrontRight.setDesiredState(swerveModuleStates[1]);
    moduleRearLeft.setDesiredState(swerveModuleStates[2]);
    moduleRearRight.setDesiredState(swerveModuleStates[3]);
  }

  // Sets the wheels into an X formation to prevent movement.  
  public void setX() {
    moduleFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    moduleFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    moduleRearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    moduleRearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    moduleFrontLeft.setDesiredState(desiredStates[0]);
    moduleFrontRight.setDesiredState(desiredStates[1]);
    moduleRearLeft.setDesiredState(desiredStates[2]);
    moduleRearRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    moduleFrontLeft.setDesiredState(swerveModuleStates[0]);
    moduleFrontRight.setDesiredState(swerveModuleStates[1]);
    moduleRearLeft.setDesiredState(swerveModuleStates[2]);
    moduleRearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    moduleFrontLeft.resetEncoders();
    moduleRearLeft.resetEncoders();
    moduleFrontRight.resetEncoders();
    moduleRearRight.resetEncoders();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        moduleFrontLeft.getPosition(),
        moduleFrontRight.getPosition(),
        moduleRearLeft.getPosition(),
        moduleRearRight.getPosition()
      }, 
      pose);
  }

  public void align() {
    PIDController controller = new PIDController(0.01, 0, 0);
    controller.setSetpoint(0);
    System.out.println(controller.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), controller.getSetpoint()));
    drive(0, 0, controller.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), controller.getSetpoint()));
  }

  public void align180() {
    PIDController controller = new PIDController(0.01, 0, 0);
    controller.setSetpoint(180);
    System.out.println(controller.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), controller.getSetpoint()));
    drive(0, 0, controller.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), controller.getSetpoint()));
  }

  public void setFieldRelative(boolean fieldRelative) {
    this.isFieldRelative = fieldRelative;
  }

  public boolean isFieldRelative() {
    return isFieldRelative;
  }

  public enum FieldLocation {
    OPEN,
    BLU_GRID_1,
    BLU_GRID_2,
    BLU_GRID_3,
    RED_GRID_1,
    RED_GRID_2,
    RED_GRID_3,
    SUBSTATION_1,
    SUBSTATION_2
  }

}
