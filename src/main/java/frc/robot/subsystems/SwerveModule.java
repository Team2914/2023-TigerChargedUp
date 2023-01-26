package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.SModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/* Note for future programmers: everything is handled by these SwerveModule classes so don't worry about the 
   math and science and stuff. Just copy and paste shit, they're all modular. 
   BUT REMEMBER TO UPDATE THE CONFIGURATION FILE! (Constants.java) */

// SwerveModule - the module
public class SwerveModule {
    // Hardware stuff 
    private CANSparkMax m_drive;
    private CANSparkMax m_rotation;
    private AbsoluteEncoder en_rotationEncoder;
    private RelativeEncoder en_driveEncoder;
    private SparkMaxPIDController pid_drive;
    private SparkMaxPIDController pid_rotation;

    private double fChassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int iDriveCANID, int iRotateCANID, double fChassisAngularOffset) {
        m_drive = new CANSparkMax(iDriveCANID, MotorType.kBrushless);
        m_rotation = new CANSparkMax(iRotateCANID, MotorType.kBrushless);

        // Restore to factory defaults
        m_drive.restoreFactoryDefaults();
        m_rotation.restoreFactoryDefaults();

        // Getting encoders and PID controller
        en_driveEncoder = m_drive.getEncoder();
        en_rotationEncoder = m_rotation.getAbsoluteEncoder(Type.kDutyCycle);
        pid_drive = m_drive.getPIDController();
        pid_rotation = m_rotation.getPIDController();
        pid_drive.setFeedbackDevice(en_driveEncoder);
        pid_rotation.setFeedbackDevice(en_rotationEncoder);

        // Convert rotations and RPM to meters and meters per second for WPILib
        en_driveEncoder.setPositionConversionFactor(SModuleConstants.kDrivingEncoderPositionFactor);
        en_driveEncoder.setVelocityConversionFactor(SModuleConstants.kDrivingEncoderVelocityFactor);

        // Convert encoder stuff to radians and radians per second
        en_rotationEncoder.setPositionConversionFactor(SModuleConstants.kTurningEncoderPositionFactor);
        en_rotationEncoder.setVelocityConversionFactor(SModuleConstants.kTurningEncoderVelocityFactor);

        en_rotationEncoder.setInverted(SModuleConstants.kTurningEncoderInverted);

        pid_rotation.setPositionPIDWrappingEnabled(true);
        pid_rotation.setPositionPIDWrappingMaxInput(SModuleConstants.kTurningEncoderPositionPIDMaxInput);
        pid_rotation.setPositionPIDWrappingMinInput(SModuleConstants.kTurningEncoderPositionPIDMinInput);

        // Set PID gains for drive motor
        pid_drive.setP(SModuleConstants.kDrivingP);
        pid_drive.setI(SModuleConstants.kDrivingI);
        pid_drive.setD(SModuleConstants.kDrivingD);
        pid_drive.setFF(SModuleConstants.kDrivingFF);
        pid_drive.setOutputRange(SModuleConstants.kDrivingMinOutput, SModuleConstants.kDrivingMaxOutput);

        // Set PID gains for turning motor
        pid_rotation.setP(SModuleConstants.kTurningP);
        pid_rotation.setI(SModuleConstants.kTurningI);
        pid_rotation.setD(SModuleConstants.kTurningD);
        pid_rotation.setFF(SModuleConstants.kTurningFF);
        pid_rotation.setOutputRange(SModuleConstants.kTurningMinOutput, SModuleConstants.kTurningMaxOutput);

        m_drive.setIdleMode(SModuleConstants.kDrivingMotorIdleMode);
        m_rotation.setIdleMode(SModuleConstants.kTurningMotorIdleMode);
        m_drive.setSmartCurrentLimit(SModuleConstants.kDrivingMotorCurrentLimit);
        m_rotation.setSmartCurrentLimit(SModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will 
        // maintain the above configurations.
        m_drive.burnFlash();
        m_rotation.burnFlash();

        this.fChassisAngularOffset = fChassisAngularOffset;
        desiredState.angle = new Rotation2d(en_rotationEncoder.getPosition());
        en_driveEncoder.setPosition(0);
    }

    // Returns the current state of the module
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(en_driveEncoder.getVelocity(),
                                        new Rotation2d(en_rotationEncoder.getPosition() - fChassisAngularOffset));
    }

    // Returns the current position of the module
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(en_driveEncoder.getPosition(),
                                        new Rotation2d(en_rotationEncoder.getPosition() - fChassisAngularOffset));
    }

    // Sets the desired state for the module
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(fChassisAngularOffset));
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                                                    new Rotation2d(en_rotationEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        pid_drive.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        pid_rotation.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    
        this.desiredState = desiredState;
    }
    
    // Zeroes all the SwerveModule encoders. 
    public void resetEncoders() {
        en_driveEncoder.setPosition(0);
    }
}
