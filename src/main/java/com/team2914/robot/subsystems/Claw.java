package com.team2914.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2914.robot.Constants.ClawConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static Claw instance = null;
    private final CANSparkMax rotateMotor;
    private final RelativeEncoder rotateEncoder;
    private final SparkMaxPIDController rotatePID;
    private final CANSparkMax intakeMotor;

    private Claw() {
        rotateMotor = new CANSparkMax(ClawConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();
        intakeMotor = new CANSparkMax(ClawConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();

        rotateEncoder = rotateMotor.getEncoder();
        rotatePID = rotateMotor.getPIDController();
        rotatePID.setFeedbackDevice(rotateEncoder);
        rotatePID.setP(0.05);
        rotatePID.setI(0);
        rotatePID.setD(0);
        rotatePID.setOutputRange(-1, 1);

        rotateMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotate motor position", rotateEncoder.getPosition());
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setRotatePosition(int pos) {
        rotatePID.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

}
