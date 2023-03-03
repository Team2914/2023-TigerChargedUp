package com.team2914.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2914.robot.Constants.LiftConstants;
import com.team2914.robot.utils.MathUtil;

public class Lift extends SubsystemBase {
    private static Lift instance = null;
    private final CANSparkMax shoulderMotor;
    private final RelativeEncoder shoulderMotorEncoder;
    private final SparkMaxPIDController shoulderMotorPID;
    private final CANSparkMax elbowMotor;
    private final RelativeEncoder elbowMotorEncoder;
    private final SparkMaxPIDController elbowMotorPID;
    private final CANSparkMax shoulderFollowMotor;
    private final RelativeEncoder shoulderFollowMotorEncoder;
    private final CANSparkMax elbowFollowMotor;
    private final RelativeEncoder elbowFollowMotorEncoder;
    
    private double armX = 0;
    private double armY = 0;
    private double shoulderAngle = 0;
    private double elbowAngle = 0;

    private Lift() {
        shoulderMotor = new CANSparkMax(LiftConstants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless);
        shoulderMotor.restoreFactoryDefaults();
        elbowMotor = new CANSparkMax(LiftConstants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        shoulderFollowMotor = new CANSparkMax(LiftConstants.SHOULDER_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        shoulderFollowMotor.restoreFactoryDefaults();
        elbowFollowMotor = new CANSparkMax(LiftConstants.ELBOW_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        elbowFollowMotor.restoreFactoryDefaults();

        shoulderMotorEncoder = shoulderMotor.getEncoder();
        shoulderMotorEncoder.setPositionConversionFactor(42);
        shoulderMotorPID = shoulderMotor.getPIDController();
        shoulderMotorPID.setFeedbackDevice(shoulderMotorEncoder);
        shoulderMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderMotorPID.setI(LiftConstants.SHOULDER_PID.kP);
        shoulderMotorPID.setD(LiftConstants.SHOULDER_PID.kP);
        shoulderMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        
        elbowMotorEncoder = elbowMotor.getEncoder();
        elbowMotorEncoder.setPositionConversionFactor(42);
        elbowMotorPID = elbowMotor.getPIDController();
        elbowMotorPID.setFeedbackDevice(elbowMotorEncoder);
        elbowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setI(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setD(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        elbowMotor.setIdleMode(IdleMode.kBrake);

        shoulderFollowMotorEncoder = shoulderFollowMotor.getEncoder();
        shoulderFollowMotorEncoder.setPositionConversionFactor(42);
        elbowFollowMotorEncoder = elbowFollowMotor.getEncoder();
        elbowFollowMotorEncoder.setPositionConversionFactor(42);
        shoulderFollowMotor.follow(shoulderMotor);
        elbowFollowMotor.follow(elbowMotor);

        resetArm();
        //moveArm(2, 2);
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder joint encoder position", shoulderMotorEncoder.getPosition());
        SmartDashboard.putNumber("Elbow joint encoder position", elbowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder angle", shoulderAngle);
        SmartDashboard.putNumber("Elbow angle", shoulderAngle + elbowAngle);
        SmartDashboard.putNumber("Arm X", armX);
        SmartDashboard.putNumber("Arm Y", armY);
    }

    public void setMotorPosition(double position) {
        shoulderMotorPID.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void moveArm(double dx, double dy) {
        armX += dx * 10;
        armY += dy * 10;

        double shoulderLength2 = LiftConstants.SHOULDER_LENGTH * LiftConstants.SHOULDER_LENGTH;
        double elbowLength2 = LiftConstants.ELBOW_LENGTH * LiftConstants.ELBOW_LENGTH;
        double a = (armX * armX) + (armY * armY) - shoulderLength2 - elbowLength2;
        double b = 2 * LiftConstants.SHOULDER_LENGTH * LiftConstants.ELBOW_LENGTH;
        elbowAngle = Math.acos(a / b) * -1;

        double a1 = LiftConstants.ELBOW_LENGTH * Math.sin(elbowAngle);
        double b1 = LiftConstants.SHOULDER_LENGTH + LiftConstants.ELBOW_LENGTH * Math.cos(elbowAngle);
        shoulderAngle = Math.atan(armY / armX) - Math.atan(a1 / b1);

        shoulderMotorPID.setReference(
            MathUtil.degreesToEncoders(
                shoulderAngle, 
                LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO, 
                42), 
            CANSparkMax.ControlType.kPosition);

        shoulderMotorPID.setReference(
            MathUtil.degreesToEncoders(
                shoulderAngle + elbowAngle, 
                LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO, 
                42), 
            CANSparkMax.ControlType.kPosition);
    }

    public void resetEncoders() {
        
    }

    public void resetArm() {
        shoulderAngle = 0;
        elbowAngle = 0;
        shoulderMotorEncoder.setPosition(
            MathUtil.degreesToEncoders(
                shoulderAngle, 
                LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO,
                42)
            );
        elbowMotorEncoder.setPosition(
            MathUtil.degreesToEncoders(
                shoulderAngle, 
                LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO,
                42)
            );
        shoulderMotorPID.setReference(shoulderMotorEncoder.getPosition(), ControlType.kPosition);
        elbowMotorPID.setReference(elbowMotorEncoder.getPosition(), ControlType.kPosition);
    }
}
