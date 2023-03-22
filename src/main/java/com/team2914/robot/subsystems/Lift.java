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

import com.team2914.lib.TigerSparkMAX;
import com.team2914.robot.RobotContainer;
import com.team2914.robot.Constants.LiftConstants;
import com.team2914.robot.utils.ClawState;
import com.team2914.robot.utils.MathUtil;

public class Lift extends SubsystemBase {
    private static Lift instance = null;
    private final CANSparkMax shoulderSpark;
    private final SparkMaxPIDController shoulderPID;
    private final RelativeEncoder shoulderEncoder;

    private final CANSparkMax elbowSpark;
    private final SparkMaxPIDController elbowPID;
    private final RelativeEncoder elbowEncoder;

    private final CANSparkMax shoulderFollowSpark;
    private final SparkMaxPIDController shoulderFollowPID;
    private final RelativeEncoder shoulderFollowEncoder;

    private final CANSparkMax elbowFollowSpark;
    private final SparkMaxPIDController elbowFollowPID;
    private final RelativeEncoder elbowFollowEncoder;
    
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
        shoulderSpark = new CANSparkMax(LiftConstants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless);
        shoulderPID = shoulderSpark.getPIDController();
        TigerSparkMAX.init(
            shoulderSpark,
            shoulderPID,
            LiftConstants.SHOULDER_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        elbowSpark = new CANSparkMax(LiftConstants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);
        elbowPID = elbowSpark.getPIDController();
        TigerSparkMAX.init(
            elbowSpark,
            elbowPID,
            LiftConstants.ELBOW_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);
            
        shoulderFollowSpark = new CANSparkMax(LiftConstants.SHOULDER_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        shoulderFollowPID = shoulderFollowSpark.getPIDController();
        TigerSparkMAX.init(
            shoulderFollowSpark,
            shoulderFollowPID,
            LiftConstants.SHOULDER_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        elbowFollowSpark = new CANSparkMax(LiftConstants.ELBOW_FOLLOW_MOTOR_CAN_ID, MotorType.kBrushless);
        elbowFollowPID = elbowFollowSpark.getPIDController();
        TigerSparkMAX.init(
            elbowFollowSpark,
            elbowFollowPID,
            LiftConstants.ELBOW_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        shoulderEncoder = shoulderSpark.getEncoder();
        shoulderEncoder.setPositionConversionFactor(42);
        shoulderPID.setFeedbackDevice(shoulderEncoder);

        elbowEncoder = elbowSpark.getEncoder();
        elbowEncoder.setPositionConversionFactor(42);
        elbowPID.setFeedbackDevice(elbowEncoder);

        shoulderFollowEncoder = shoulderFollowSpark.getEncoder();
        shoulderFollowEncoder.setPositionConversionFactor(42);
        shoulderFollowPID.setFeedbackDevice(shoulderFollowEncoder);

        elbowFollowEncoder = elbowFollowSpark.getEncoder();
        elbowFollowEncoder.setPositionConversionFactor(42);
        elbowFollowPID.setFeedbackDevice(elbowFollowEncoder);
        
        shoulderSpark.setInverted(true);
        elbowFollowSpark.setInverted(true);

        cubeMode = false;

        resetArm();
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    public void runLift(double val){
        elbowSpark.set(val*0.1);
        elbowFollowSpark.set(val*0.1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Arm/Shoulder joint encoder position", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("/Arm/Elbow joint encoder position", elbowEncoder.getPosition());
        SmartDashboard.putNumber("/Arm/Shoulder joint follow encoder position", shoulderFollowEncoder.getPosition());
        SmartDashboard.putNumber("/Arm/Elbow joint follow encoder position", elbowFollowEncoder.getPosition());
        SmartDashboard.putNumber("/Arm/Shoulder angle", Math.toDegrees(shoulderAngle));
        SmartDashboard.putNumber("/Arm/Elbow angle", Math.toDegrees(shoulderAngle + elbowAngle));
        SmartDashboard.putNumber("/Arm/Arm X", armX);
        SmartDashboard.putNumber("/Arm/Arm Y", armY);
        SmartDashboard.putNumber("/Arm/Elbow target position", elbowTargetPosition);
        SmartDashboard.putNumber("/Arm/Shoulder target position", shoulderTargetPosition);
        SmartDashboard.putNumber("/Arm/Elbow Output", elbowSpark.getAppliedOutput());

        if (OperatorController.getInstance().getSlider() > 0) {
            cubeMode = true;
        } else {
            cubeMode = false;
        }
    }

    public void moveArm(double dx, double dy) {
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
        shoulderPID.setReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        shoulderFollowPID.setReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);

        elbowPID.setReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        elbowFollowPID.setReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
    }

    public void setArmHigh() {
        shoulderPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        if (ClawState.hasCube()) {
            armX = 100;
            armY = 180;
        } else {
            armX = 110;
            armY = 210;
        }

        ClawState.liftLevel = 3;
        
    }

    public void setArmMid() {
        shoulderPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        if (ClawState.hasCube()) {
            armX = 80;
            armY = 110;
        } else {
            armX = 80;
            armY = 210;
        }

        ClawState.liftLevel = 2;
    }

    public void liftGamePiece() {
        shoulderPID.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowPID.setP(LiftConstants.SHOULDER_PID.kP);
        elbowPID.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowPID.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        armX = 35;
        armY = 60;

        ClawState.liftLevel = 1;
    }

    public void setArmLow() {
        shoulderPID.setP(LiftConstants.SHOULDER_PID.kP * 0.5);
        shoulderFollowPID.setP(LiftConstants.SHOULDER_PID.kP * 0.5);
        elbowPID.setP(LiftConstants.ELBOW_PID.kP * 0.01);
        
        elbowFollowPID.setP(LiftConstants.ELBOW_PID.kP * 0.01);
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

        shoulderEncoder.setPosition(shoulderTargetPosition);
        elbowEncoder.setPosition(elbowTargetPosition);
            
        shoulderFollowEncoder.setPosition(shoulderTargetPosition);
        elbowFollowEncoder.setPosition(elbowTargetPosition);
        shoulderPID.setReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowPID.setReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
        shoulderFollowPID.setReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowFollowPID.setReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
    }

    public void setCoast() {
        shoulderSpark.setIdleMode(IdleMode.kCoast);
        shoulderFollowSpark.setIdleMode(IdleMode.kCoast);
        elbowSpark.setIdleMode(IdleMode.kCoast);
        elbowFollowSpark.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake() {
        shoulderSpark.setIdleMode(IdleMode.kBrake);
        shoulderFollowSpark.setIdleMode(IdleMode.kBrake);
        elbowSpark.setIdleMode(IdleMode.kBrake);
        elbowFollowSpark.setIdleMode(IdleMode.kBrake);
    }
}
