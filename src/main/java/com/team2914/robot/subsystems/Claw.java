package com.team2914.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    private final CANSparkMax intakeMotor;

    private int rotationPosition = 0;

    public boolean closed = false;

    private final int CLOSED_POS = -7;
    private final int OPEN_POS = 0;


    private Claw() {
        intakeMotor = new CANSparkMax(ClawConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);

        rotateMotor = new CANSparkMax(ClawConstants.ROTATE_CAN_ID, MotorType.kBrushed);
        rotateMotor.setIdleMode(IdleMode.kBrake);
        rotateMotor.setSmartCurrentLimit(15);
        
    }


    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw closed", closed);
        SmartDashboard.putNumber("Claw Reference", rotationPosition);
        SmartDashboard.putNumber("Claw Rotation Current", rotateMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Rotation Voltage", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake Rotation Current", intakeMotor.getOutputCurrent());
    }

    public void closeClaw() {
        set(-1.0);
        closed = true;
    }

    public void set(double val) {
        rotateMotor.set(val);
    }

    public void openClaw() {
        //System.out.print("running open claw");
        set(0.75);
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



}
