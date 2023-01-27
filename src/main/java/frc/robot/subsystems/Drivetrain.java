package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Drivetrain extends SubsystemBase {
    private SwerveModule m_frontLeft;
    private SwerveModule m_backLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backRight;
    private AHRS gyro;
    private SwerveDrivePoseEstimator odometry;
    private SwerveDriveKinematics kinematics;
    private boolean bFieldRelative;

    private PhotonAprilTags photon;

    public Drivetrain() {
        // Swerve modules
        m_frontLeft = new SwerveModule(
            DrivetrainConstants.kFrontLeftDrivePort, 
            DrivetrainConstants.kFrontLeftTurningPort, 
            DrivetrainConstants.kFrontLeftChassisAngularOffset);
        m_backLeft = new SwerveModule(
            DrivetrainConstants.kBackLeftDrivePort, 
            DrivetrainConstants.kBackLeftTurningPort, 
            DrivetrainConstants.kBackLeftChassisAngularOffset);
        m_frontRight = new SwerveModule(
            DrivetrainConstants.kFrontRightDrivePort, 
            DrivetrainConstants.kFrontRightTurningPort, 
            DrivetrainConstants.kFrontRightChassisAngularOffset);
        m_backRight = new SwerveModule(
            DrivetrainConstants.kBackRightDrivePort, 
            DrivetrainConstants.kBackRightTurningPort, 
            DrivetrainConstants.kBackRightChassisAngularOffset);

        bFieldRelative = false;    
        // NavX
        gyro = new AHRS(SerialPort.Port.kUSB);
        // Swerve kinematics
        kinematics = DrivetrainConstants.kDriveKinematics;
        // Odometry / pose estimator
        odometry = new SwerveDrivePoseEstimator(
            kinematics, 
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            new Pose2d()
        );

        // PhotonVision
        photon = new PhotonAprilTags();    
        
    }

    // Update odometry
    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );

        Optional<EstimatedRobotPose> visionResult = photon.getEstimatedRobotPose(odometry.getEstimatedPosition());

        if (!visionResult.isPresent()) return;
        
        // Update odometry if an AprilTag is found
        EstimatedRobotPose cameraPose = visionResult.get();
        odometry.addVisionMeasurement(cameraPose.estimatedPose.toPose2d(), cameraPose.timestampSeconds);
    }

    // Returns the current pose of the robot
    public Pose2d getEstimatedPos() {
        return odometry.getEstimatedPosition();
    }

    // Resets the odometry to the specified pose
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            pose
        );
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }
    
    // X dir: forward backwards
    // Y dir: side to side
    public void drive(double fXSpeed, double fYSpeed, double fRot) {
        fXSpeed *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
        fYSpeed *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
        fRot *= DrivetrainConstants.kMaxAngularSpeed;

        ChassisSpeeds fieldRelativeSpeeds = 
            ChassisSpeeds.fromFieldRelativeSpeeds(fXSpeed, fYSpeed, fRot, gyro.getRotation2d());

        SwerveModuleState[] moduleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            bFieldRelative ?  fieldRelativeSpeeds : new ChassisSpeeds(fXSpeed, fYSpeed, fRot));

        setModuleStates(moduleStates);
    }

    // Stop movement by setting the wheels to an X formation
    public void stop() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public boolean isFieldRelative() {
        return bFieldRelative;
    }

    public void setFieldRelative(boolean val) {
        bFieldRelative = val;
    }
}
