package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public class Drivetrain extends SubsystemBase {
    private SwerveModule m_frontLeft;
    private SwerveModule m_backLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backRight;
    private ADIS16470_IMU g_gyro;
    private SwerveDriveOdometry odometry;

    public Drivetrain() {
        m_frontLeft = new SwerveModule(
            DrivetrainConstants.FRONT_LEFT_DRIVE_PORT, 
            DrivetrainConstants.FRONT_LEFT_TURNING_PORT, 
            DrivetrainConstants.kFrontLeftChassisAngularOffset);
        m_backLeft = new SwerveModule(
            DrivetrainConstants.BACK_LEFT_DRIVE_PORT, 
            DrivetrainConstants.BACK_LEFT_TURNING_PORT, 
            DrivetrainConstants.kBackLeftChassisAngularOffset);
        m_frontRight = new SwerveModule(
            DrivetrainConstants.FRONT_RIGHT_DRIVE_PORT, 
            DrivetrainConstants.FRONT_RIGHT_TURNING_PORT, 
            DrivetrainConstants.kFrontRightChassisAngularOffset);
        m_backRight = new SwerveModule(
            DrivetrainConstants.BACK_RIGHT_DRIVE_PORT, 
            DrivetrainConstants.BACK_RIGHT_TURNING_PORT, 
            DrivetrainConstants.kBackRightChassisAngularOffset);

        g_gyro = new ADIS16470_IMU();

        odometry = new SwerveDriveOdometry(
            DrivetrainConstants.kDriveKinematics, 
            Rotation2d.fromDegrees(g_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
            );

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            Rotation2d.fromDegrees(g_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
            );
    }
    
}
