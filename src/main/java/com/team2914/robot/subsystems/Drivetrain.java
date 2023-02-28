// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team2914.lib.gyro.BNO055;
import com.team2914.lib.gyro.BNO055.opmode_t;
import com.team2914.lib.gyro.BNO055.vector_type_t;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team2914.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

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
    SmartDashboard.putBoolean("Field relative", isFieldRelative);

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

    if (!camResult.isPresent()) return;

    EstimatedRobotPose camPose = camResult.get();
    poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    
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
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) :
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

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //gyro.reset();
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

  public void setFieldRelative(boolean fieldRelative) {
    this.isFieldRelative = fieldRelative;
  }

  public boolean isFieldRelative() {
    return isFieldRelative;
  }

}
