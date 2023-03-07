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
    private final SparkMaxPIDController shoulderFollowMotorPID;
    private final CANSparkMax elbowFollowMotor;
    private final RelativeEncoder elbowFollowMotorEncoder;
    private final SparkMaxPIDController elbowFollowMotorPID;
    
    private double armX = 0;
    private double armY = 0;
    private double shoulderAngle = 0;
    private double elbowAngle = 0;

    private double elbowTargetPosition = 0;
    private double shoulderTargetPosition = 0;

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
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowMotorPID.setI(LiftConstants.ELBOW_PID.kP);
        elbowMotorPID.setD(LiftConstants.ELBOW_PID.kP);
        elbowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        elbowMotor.setIdleMode(IdleMode.kBrake);

        shoulderFollowMotorEncoder = shoulderFollowMotor.getEncoder();
        shoulderFollowMotorEncoder.setPositionConversionFactor(42);
        shoulderFollowMotorPID = shoulderFollowMotor.getPIDController();
        shoulderFollowMotorPID.setFeedbackDevice(shoulderFollowMotorEncoder);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setI(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setD(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        shoulderFollowMotor.setIdleMode(IdleMode.kBrake);


        elbowFollowMotorEncoder = elbowFollowMotor.getEncoder();
        elbowFollowMotorEncoder.setPositionConversionFactor(42);
        elbowFollowMotorPID = elbowFollowMotor.getPIDController();
        elbowFollowMotorPID.setFeedbackDevice(elbowFollowMotorEncoder);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setI(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setD(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        elbowFollowMotor.setIdleMode(IdleMode.kBrake);
        
        shoulderFollowMotor.setInverted(true);
        elbowFollowMotor.setInverted(true);

        /*shoulderMotorEncoder.setInverted(true);
        shoulderFollowMotorEncoder.setInverted(true);
        elbowMotorEncoder.setInverted(true);
        elbowFollowMotorEncoder.setInverted(true);*/


        //resetArm();
        //moveArm(2, 2);
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    public void resetEncoders() {
        shoulderMotorEncoder.setPosition(0);
        shoulderFollowMotorEncoder.setPosition(0);
        elbowMotorEncoder.setPosition(0);
        elbowFollowMotorEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder joint encoder position", shoulderMotorEncoder.getPosition());
        SmartDashboard.putNumber("Elbow joint encoder position", elbowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder joint follow encoder position", shoulderFollowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Elbow joint follow encoder position", elbowFollowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder angle", Math.toDegrees(shoulderAngle));
        SmartDashboard.putNumber("Elbow angle", Math.toDegrees(shoulderAngle + elbowAngle));
        SmartDashboard.putNumber("Arm X", armX);
        SmartDashboard.putNumber("Arm Y", armY);
        SmartDashboard.putNumber("Elbow Target", elbowTargetPosition);
    }

    public void setMotorPosition(double position) {
        elbowTargetPosition = MathUtil.radToEncoders(
            (Math.PI / 3)*position, 
            LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO, 
            42);
        shoulderTargetPosition = MathUtil.radToEncoders(
            (Math.PI / 9)*position, 
            LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO, 
            42);
        elbowTargetPosition -= shoulderTargetPosition;
        elbowMotorPID.setReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);

        elbowFollowMotorPID.setReference(
                elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);

            shoulderMotorPID.setReference(
                elbowTargetPosition, 
                CANSparkMax.ControlType.kPosition);
    
            shoulderFollowMotorPID.setReference(
                    elbowTargetPosition, 
                CANSparkMax.ControlType.kPosition);

        
    }

    public void moveArm(double dx, double dy) {
        armX += dx * 2;
        armY += dy * 2;

        double shoulderLength2 = LiftConstants.SHOULDER_LENGTH * LiftConstants.SHOULDER_LENGTH;
        double elbowLength2 = LiftConstants.ELBOW_LENGTH * LiftConstants.ELBOW_LENGTH;
        double a = (armX * armX) + (armY * armY) - shoulderLength2 - elbowLength2;
        double b = 2 * LiftConstants.SHOULDER_LENGTH * LiftConstants.ELBOW_LENGTH;
        elbowAngle = Math.acos(a / b) * -1;

        double a1 = LiftConstants.ELBOW_LENGTH * Math.sin(elbowAngle);
        double b1 = LiftConstants.SHOULDER_LENGTH + LiftConstants.ELBOW_LENGTH * Math.cos(elbowAngle);
        shoulderAngle = Math.atan(armY / armX) - Math.atan(a1 / b1);

        shoulderMotorPID.setReference(
            MathUtil.radToEncoders(
                shoulderAngle, 
                LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO, 
                42), 
            CANSparkMax.ControlType.kPosition);

        elbowMotorPID.setReference(
            MathUtil.radToEncoders(
                shoulderAngle + elbowAngle, 
                LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO, 
                42), 
            CANSparkMax.ControlType.kPosition);
    }

    public void resetArm() {
        shoulderAngle = Math.PI / 3;
        elbowAngle = 0;
        armX = 0;
        armY = 0;
        shoulderMotorEncoder.setPosition(
            MathUtil.radToEncoders(
                shoulderAngle, 
                LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO,
                42)
            );
        elbowMotorEncoder.setPosition(
            MathUtil.radToEncoders(
                shoulderAngle, 
                LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO,
                42)
            );
        shoulderMotorPID.setReference(shoulderMotorEncoder.getPosition(), ControlType.kPosition);
        elbowMotorPID.setReference(elbowMotorEncoder.getPosition(), ControlType.kPosition);
    }
}
