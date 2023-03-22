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
    private final TigerSparkMAX shoulderSpark;
    private final RelativeEncoder shoulderEncoder;

    private final TigerSparkMAX elbowSpark;
    private final RelativeEncoder elbowEncoder;

    private final TigerSparkMAX shoulderFollowSpark;
    private final RelativeEncoder shoulderFollowEncoder;

    private final TigerSparkMAX elbowFollowSpark;
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
        shoulderSpark = new TigerSparkMAX(
            LiftConstants.SHOULDER_MOTOR_CAN_ID, 
            MotorType.kBrushless, 
            LiftConstants.SHOULDER_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        elbowSpark = new TigerSparkMAX(
            LiftConstants.ELBOW_MOTOR_CAN_ID, 
            MotorType.kBrushless, 
            LiftConstants.ELBOW_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);
            
        shoulderFollowSpark = new TigerSparkMAX(
            LiftConstants.SHOULDER_FOLLOW_MOTOR_CAN_ID, 
            MotorType.kBrushless, 
            LiftConstants.SHOULDER_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        elbowFollowSpark = new TigerSparkMAX(
            LiftConstants.ELBOW_FOLLOW_MOTOR_CAN_ID, 
            MotorType.kBrushless, 
            LiftConstants.ELBOW_PID, 
            0, 
            LiftConstants.LIFT_MIN_OUTPUT, 
            LiftConstants.LIFT_MAX_OUTPUT, 
            IdleMode.kBrake, 
            60);

        shoulderEncoder = shoulderSpark.getRelativeEncoder();
        shoulderEncoder.setPositionConversionFactor(42);
        shoulderSpark.setPIDFeedbackDevice(shoulderEncoder);

        elbowEncoder = elbowSpark.getRelativeEncoder();
        elbowEncoder.setPositionConversionFactor(42);
        elbowSpark.setPIDFeedbackDevice(elbowEncoder);

        shoulderFollowEncoder = shoulderFollowSpark.getRelativeEncoder();
        shoulderFollowEncoder.setPositionConversionFactor(42);
        shoulderFollowSpark.setPIDFeedbackDevice(shoulderFollowEncoder);

        elbowFollowEncoder = elbowFollowSpark.getRelativeEncoder();
        elbowFollowEncoder.setPositionConversionFactor(42);
        elbowFollowSpark.setPIDFeedbackDevice(elbowFollowEncoder);
        
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
        elbowSpark.sparkMax.set(val*0.1);
        elbowFollowSpark.sparkMax.set(val*0.1);
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
        SmartDashboard.putNumber("/Arm/Elbow Output", elbowSpark.sparkMax.getAppliedOutput());

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

        shoulderSpark.setPIDReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        shoulderFollowSpark.setPIDReference(
            shoulderTargetPosition, 
            CANSparkMax.ControlType.kPosition);

        elbowSpark.setPIDReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
        elbowFollowSpark.setPIDReference(
            elbowTargetPosition, 
            CANSparkMax.ControlType.kPosition);
    }

    public void setArmHigh() {
        shoulderSpark.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowSpark.setP(LiftConstants.SHOULDER_PID.kP);
        elbowSpark.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowSpark.setP(LiftConstants.ELBOW_PID.kP);
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
        shoulderSpark.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowSpark.setP(LiftConstants.SHOULDER_PID.kP);
        elbowSpark.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowSpark.setP(LiftConstants.ELBOW_PID.kP);
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
        shoulderSpark.setP(LiftConstants.SHOULDER_PID.kP);
        shoulderFollowSpark.setP(LiftConstants.SHOULDER_PID.kP);
        elbowSpark.setP(LiftConstants.ELBOW_PID.kP);
        elbowFollowSpark.setP(LiftConstants.ELBOW_PID.kP);
        // elbowSpark.setI(LiftConstants.ELBOW_PID.kI);
        // elbowFollowSpark.setI(LiftConstants.ELBOW_PID.kI);

        armX = 35;
        armY = 60;

        ClawState.liftLevel = 1;
    }

    public void setArmLow() {
        shoulderSpark.setP(LiftConstants.SHOULDER_PID.kP * 0.5);
        shoulderFollowSpark.setP(LiftConstants.SHOULDER_PID.kP * 0.5);
        elbowSpark.setP(LiftConstants.ELBOW_PID.kP * 0.01);
        
        elbowFollowSpark.setP(LiftConstants.ELBOW_PID.kP * 0.01);
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

        shoulderSpark.setPIDReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowSpark.setPIDReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
        shoulderFollowSpark.setPIDReference(shoulderTargetPosition, CANSparkMax.ControlType.kPosition);
        elbowFollowSpark.setPIDReference(elbowTargetPosition, CANSparkMax.ControlType.kPosition);
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
