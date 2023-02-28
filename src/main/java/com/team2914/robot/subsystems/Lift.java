package com.team2914.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2914.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
    private static Lift instance = null;
    private final CANSparkMax motor;
    private final RelativeEncoder motorEncoder;
    private final SparkMaxPIDController motorPIDController;
    
    private double armX = 0;
    private double armY = 0;
    private double shoulderAngle = 0;
    private double elbowAngle = 0;

    private Lift() {
        motor = new CANSparkMax(LiftConstants.LIFT_MOTOR_CAN_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(42);
        motorPIDController = motor.getPIDController();
        motorPIDController.setFeedbackDevice(motorEncoder);
        motorPIDController.setP(LiftConstants.LIFT_P);
        motorPIDController.setI(LiftConstants.LIFT_I);
        motorPIDController.setD(LiftConstants.LIFT_D);
        motorPIDController.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);

        motor.setIdleMode(IdleMode.kBrake);

        setArmTarget(2, 2);
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift encoder position", motorEncoder.getPosition());
        SmartDashboard.putNumber("Arm X", armX);
        SmartDashboard.putNumber("Arm Y", armY);
    }

    public void setMotorPosition(double position) {
        motorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setArmTarget(double x, double y) {
        armX += x * 10;
        armY += y * 10;

        double shoulderLength2 = LiftConstants.SHOULDER_LENGTH * LiftConstants.SHOULDER_LENGTH;
        double elbowLength2 = LiftConstants.ELBOW_LENGTH * LiftConstants.ELBOW_LENGTH;
        double a = (armX * armX) + (armY * armY) - shoulderLength2 - elbowLength2;
        double b = 2 * LiftConstants.SHOULDER_LENGTH * LiftConstants.ELBOW_LENGTH;
        elbowAngle = Math.acos(a / b) * -1;

        double a1 = LiftConstants.ELBOW_LENGTH * Math.sin(elbowAngle);
        double b1 = LiftConstants.SHOULDER_LENGTH + LiftConstants.ELBOW_LENGTH * Math.cos(elbowAngle);
        shoulderAngle = Math.atan(armY / armX) - Math.atan(a1 / b1);
    }

    public void resetEncoders() {
        motorEncoder.setPosition(0);
        motorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }
}
