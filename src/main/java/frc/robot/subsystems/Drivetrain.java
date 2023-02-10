// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import frc.robot.utils.BNO055;
import frc.robot.utils.BNO055.opmode_t;
import frc.robot.utils.BNO055.vector_type_t;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule moduleFrontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule moduleFrontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule moduleRearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule moduleRearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  //private final AHRS gyro;
  private final BNO055 gyro;
  private final PhotonAprilTags photon = new PhotonAprilTags();
  private Field2d field = new Field2d();
  SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    //gyro = new AHRS(I2C.Port.kMXP);
    gyro = BNO055.getInstance(opmode_t.OPERATION_MODE_IMUPLUS, vector_type_t.VECTOR_EULER);

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
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

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Connected?", gyro.isSensorPresent());
    //System.out.println("Calibrating? " + gyro.isCalibrating());
    SmartDashboard.putBoolean("Gyro calibrated?", gyro.isCalibrated());
    SmartDashboard.putNumber("Heading", gyro.getHeading());
    SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());

    poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            moduleFrontLeft.getPosition(),
            moduleFrontRight.getPosition(),
            moduleRearLeft.getPosition(),
            moduleRearRight.getPosition()
        });
    field.setRobotPose(poseEstimator.getEstimatedPosition());    

    Optional<EstimatedRobotPose> camResult = photon.getEstimatedRobotPose(poseEstimator.getEstimatedPosition());

    if (!camResult.isPresent()) return;

    EstimatedRobotPose camPose = camResult.get();
    poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    //var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    //SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) :
      new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

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
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    moduleFrontLeft.setDesiredState(desiredStates[0]);
    moduleFrontRight.setDesiredState(desiredStates[1]);
    moduleRearLeft.setDesiredState(desiredStates[2]);
    moduleRearRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

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

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  /*public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }*/

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  /*public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }*/
}
