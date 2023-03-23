package com.team2914.robot.subsystems;

import java.text.NumberFormat.Style;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team2914.robot.RobotContainer;
import com.team2914.robot.Constants.LiftConstants;
import com.team2914.robot.utils.ClawState;
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
    
    private final double ARM_INIT_X = 11;
    private final double ARM_INIT_Y = 18;
    private double armX = 0;
    private double armY = 0;
    private double shoulderAngle = 0;
    private double elbowAngle = 0;

    private double elbowTargetPosition = 0;
    private double shoulderTargetPosition = 0;

    private final double StartShoulderAngle = Math.toRadians(35);
    private final double StartElbowAngle = -Math.PI;

    private boolean cubeMode;

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
        shoulderMotorPID.setI(LiftConstants.SHOULDER_PID.kI);
        shoulderMotorPID.setD(LiftConstants.SHOULDER_PID.kD);
        shoulderMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotor.setSmartCurrentLimit(60);
        
        elbowMotorEncoder = elbowMotor.getEncoder();
        elbowMotorEncoder.setPositionConversionFactor(42);
        elbowMotorPID = elbowMotor.getPIDController();
        elbowMotorPID.setFeedbackDevice(elbowMotorEncoder);
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowMotorPID.setI(LiftConstants.ELBOW_PID.kI);
        elbowMotorPID.setD(LiftConstants.ELBOW_PID.kD);
        elbowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(60);

        shoulderFollowMotorEncoder = shoulderFollowMotor.getEncoder();
        shoulderFollowMotorEncoder.setPositionConversionFactor(42);
        shoulderFollowMotorPID = shoulderFollowMotor.getPIDController();
        shoulderFollowMotorPID.setFeedbackDevice(shoulderFollowMotorEncoder);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setI(LiftConstants.SHOULDER_PID.kI);
        shoulderFollowMotorPID.setD(LiftConstants.SHOULDER_PID.kD);
        shoulderFollowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        shoulderFollowMotor.setIdleMode(IdleMode.kBrake);
        shoulderFollowMotor.setSmartCurrentLimit(60);

        elbowFollowMotorEncoder = elbowFollowMotor.getEncoder();
        elbowFollowMotorEncoder.setPositionConversionFactor(42);
        elbowFollowMotorPID = elbowFollowMotor.getPIDController();
        elbowFollowMotorPID.setFeedbackDevice(elbowFollowMotorEncoder);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setI(LiftConstants.ELBOW_PID.kI);
        elbowFollowMotorPID.setD(LiftConstants.ELBOW_PID.kD);
        elbowFollowMotorPID.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);
        elbowFollowMotor.setIdleMode(IdleMode.kBrake);
        elbowFollowMotor.setSmartCurrentLimit(60);
        
        shoulderMotor.setInverted(true);
        elbowFollowMotor.setInverted(true);

        cubeMode = false;

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
        SmartDashboard.putNumber("Shoulder joint follow encoder position", shoulderFollowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Elbow joint follow encoder position", elbowFollowMotorEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder angle", Math.toDegrees(shoulderAngle));
        SmartDashboard.putNumber("Elbow angle", Math.toDegrees(shoulderAngle + elbowAngle));
        SmartDashboard.putNumber("Arm X", armX);
        SmartDashboard.putNumber("Arm Y", armY);
        SmartDashboard.putNumber("Elbow Target", elbowTargetPosition);
        SmartDashboard.putNumber("Elbow Applied Output", elbowMotor.getAppliedOutput());
        SmartDashboard.putNumber("ELbow Current", elbowMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow Follower Applied Output", elbowFollowMotor.getAppliedOutput());
        SmartDashboard.putNumber("ELbow Follower Current", elbowFollowMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elbow Temp", elbowFollowMotor.getMotorTemperature());

        SmartDashboard.putNumber("Shoulder target", shoulderTargetPosition);
        SmartDashboard.putNumber("Shoulder Current", shoulderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shoulder Follower Current", shoulderFollowMotor.getOutputCurrent());
        SmartDashboard.putNumber("Operator controller slider", OperatorController.getInstance().getSlider());

        if (OperatorController.getInstance().getSlider() > 0) {
            cubeMode = true;
        } else {
            cubeMode = false;
        }
    }

    public void moveArm(double dx, double dy) {
        /*double travelA = Math.toRadians(80);
        double dx = Math.cos(travelA)*dH;
        double dy = Math.sin(travelA)*dH;*/
        armX += dx;
        armY += dy;
        if (armX < ARM_INIT_X) {
            armX = ARM_INIT_X;
        }
        if (armY < ARM_INIT_Y) {
            armY = ARM_INIT_Y;
        }
        if (armX > 400) {
            armX = 400;
        }
        if (armY > 400) {
            armY = 400;
        }

        double shoulderLength2 = LiftConstants.SHOULDER_LENGTH * LiftConstants.SHOULDER_LENGTH;
        double elbowLength2 = LiftConstants.ELBOW_LENGTH * LiftConstants.ELBOW_LENGTH;
        double a = (armX * armX) + (armY * armY) - shoulderLength2 - elbowLength2;
        double b = 2 * LiftConstants.SHOULDER_LENGTH * LiftConstants.ELBOW_LENGTH;
        elbowAngle = Math.acos(a / b) * -1;

        double a1 = LiftConstants.ELBOW_LENGTH * Math.sin(elbowAngle);
        double b1 = LiftConstants.SHOULDER_LENGTH + LiftConstants.ELBOW_LENGTH * Math.cos(elbowAngle);
        shoulderAngle = Math.PI - (Math.atan(armY / armX) - Math.atan(a1 / b1));

        
        shoulderTargetPosition = MathUtil.radToEncoders(
            shoulderAngle, 
            LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO, 
            42);

        elbowTargetPosition = MathUtil.radToEncoders(
            (shoulderAngle + elbowAngle), 
            LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO, 
            42);

        shoulderMotorPID.setReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        shoulderFollowMotorPID.setReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);

        elbowMotorPID.setReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        elbowFollowMotorPID.setReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
    }

    public void setArmHigh() {
        shoulderMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        if (ClawState.hasCube()) {
            armX = 100;
            armY = 180;
        } else {
            armX = 100;
            armY = 230;
        }

        ClawState.liftLevel = 3;
        
    }

    public void setArmMid() {
        shoulderMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        if (ClawState.hasCube()) {
            armX = 80;
            armY = 110;
        } else {
            armX = 90;
            armY = 205;
        }

        ClawState.liftLevel = 2;
    }

    public void liftGamePiece() {
        shoulderMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        armX = 35;
        armY = 60;

        ClawState.liftLevel = 1;
    }

    public void setArmLow() {
        shoulderMotorPID.setP(LiftConstants.SHOULDER_PID.kP*0.5);
        shoulderFollowMotorPID.setP(LiftConstants.SHOULDER_PID.kP*0.5);
        elbowMotorPID.setP(LiftConstants.ELBOW_PID.kP*0.01);
        elbowFollowMotorPID.setP(LiftConstants.ELBOW_PID.kP*0.01);
        // elbowSpark.setI(0);
        // elbowFollowSpark.setI(0); 

        armX = ARM_INIT_X;
        armY = ARM_INIT_Y;

        ClawState.liftLevel = 0;
    }

    public void setCubeMode(boolean cubeMode) {
        this.cubeMode = cubeMode;
    }

    public boolean getCubeMode() {
        return cubeMode;
    }

    public void resetArm() {
        shoulderAngle = StartShoulderAngle;
        elbowAngle = StartElbowAngle;
        armX = ARM_INIT_X;
        armY = ARM_INIT_Y;
        shoulderTargetPosition = MathUtil.radToEncoders(
            shoulderAngle, 
            LiftConstants.SHOULDER_GEAR_RATIO * LiftConstants.SHOULDER_SPROCKET_RATIO,
            42);
        elbowTargetPosition = MathUtil.radToEncoders(
            (shoulderAngle + elbowAngle), 
            LiftConstants.ELBOW_GEAR_RATIO * LiftConstants.ELBOW_SPROCKET_RATIO,
            42);

        shoulderMotorEncoder.setPosition(shoulderTargetPosition);
        elbowMotorEncoder.setPosition(elbowTargetPosition);
            
        shoulderFollowMotorEncoder.setPosition(shoulderTargetPosition);
        elbowFollowMotorEncoder.setPosition(elbowTargetPosition);

        shoulderMotorPID.setReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowMotorPID.setReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
        shoulderFollowMotorPID.setReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowFollowMotorPID.setReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
    }

    public void setCoast() {
        shoulderMotor.setIdleMode(IdleMode.kCoast);
        shoulderFollowMotor.setIdleMode(IdleMode.kCoast);
        elbowMotor.setIdleMode(IdleMode.kCoast);
        elbowFollowMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderFollowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowFollowMotor.setIdleMode(IdleMode.kBrake);
    }
}