package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Drivetrain extends SubsystemBase {
    private SwerveModule m_frontLeft;
    private SwerveModule m_backLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backRight;
    private AHRS gyro;
    private SwerveDriveOdometry odometry;
    private boolean bFieldRelative;

    public Drivetrain() {
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

        gyro = new AHRS(SerialPort.Port.kUSB);

        odometry = new SwerveDriveOdometry(
            DrivetrainConstants.kDriveKinematics, 
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
            );

        bFieldRelative = false;
    }

    // Update odometry
    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
            );
    }

    // Returns the current pose of the robot
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    // Resets the odometry to the specified pose
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            pose);
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
            ChassisSpeeds.fromFieldRelativeSpeeds(fXSpeed, fYSpeed, fRot, Rotation2d.fromDegrees(gyro.getAngle()));

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
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
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
