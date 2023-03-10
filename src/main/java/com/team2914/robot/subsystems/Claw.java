package com.team2914.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
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

    private int rotationPosition = 0;

    private boolean closed = false;

    private final int CLOSED_POS = -7;
    private final int OPEN_POS = 0;


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

        rotateEncoder.setPosition(OPEN_POS);
        rotatePID.setReference(OPEN_POS, ControlType.kPosition);
    }

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw rotate motor position", rotateEncoder.getPosition());
        SmartDashboard.putBoolean("Claw closed", closed);
        SmartDashboard.putNumber("Claw Reference", rotationPosition);
        SmartDashboard.putNumber("Claw Rotation Voltage", rotateMotor.getAppliedOutput());
        SmartDashboard.putNumber("Claw Rotation Current", rotateMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Rotation Voltage", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake Rotation Current", intakeMotor.getOutputCurrent());
    }

    public void closeClaw() {
        rotatePID.setReference(CLOSED_POS, com.revrobotics.CANSparkMax.ControlType.kPosition);
        rotationPosition = CLOSED_POS;
        closed = true;
    }

    public void openClaw() {
        rotatePID.setReference(OPEN_POS, com.revrobotics.CANSparkMax.ControlType.kPosition);
        rotationPosition = OPEN_POS;
        closed = false;
    }

    public void setIntakeSpeed(double speed) {
        if (intakeMotor.getOutputCurrent() > 10) {
            SmartDashboard.putBoolean("Intake Motor Current Trip", true);
            intakeMotor.set(0.0);
        } else {
            SmartDashboard.putBoolean("Intake Motor Current Trip", false);
            intakeMotor.set(speed);
        }
    }

    public void rotateArms(double speed){
        rotateMotor.set(speed);
    }

    public void changeRotatePosition(int dir) {
        System.out.println("Changing Claw Position");
        int posChange = dir;
        rotationPosition += posChange;
        setRotatePosition(rotationPosition);
    }

    public void setRotatePosition(int pos) {
        rotatePID.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

}
