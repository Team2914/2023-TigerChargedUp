package com.team2914.lib;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class TigerSparkMAX {
    public final CANSparkMax sparkMax;
    public final SparkMaxPIDController sparkPID;

    private double pidSetpoint;

    
    public TigerSparkMAX(
        int canID, 
        MotorType motorType, 
        PIDConstants pidConstants, 
        double ffGain, 
        double minOutput, 
        double maxOutput, 
        IdleMode idleMode,
        int currentLimit) {

        sparkMax = new CANSparkMax(canID, motorType);
        sparkPID = sparkMax.getPIDController();

        sparkMax.restoreFactoryDefaults();
        sparkMax.setIdleMode(idleMode);
        sparkMax.setSmartCurrentLimit(currentLimit);

        sparkPID.setP(pidConstants.kP);
        sparkPID.setI(pidConstants.kI);
        sparkPID.setD(pidConstants.kD);
        sparkPID.setFF(ffGain);
        sparkPID.setOutputRange(minOutput, maxOutput);
    }

    public static void init(
        CANSparkMax sparkMax,
        SparkMaxPIDController sparkPID,
        PIDConstants pidConstants, 
        double ffGain, 
        double minOutput, 
        double maxOutput, 
        IdleMode idleMode,
        int currentLimit) {

        sparkMax.restoreFactoryDefaults();
        sparkMax.setIdleMode(idleMode);
        sparkMax.setSmartCurrentLimit(currentLimit);

        sparkPID.setP(pidConstants.kP);
        sparkPID.setI(pidConstants.kI);
        sparkPID.setD(pidConstants.kD);
        sparkPID.setFF(ffGain);
        sparkPID.setOutputRange(minOutput, maxOutput);
    }

    public RelativeEncoder getRelativeEncoder() {
        return sparkMax.getEncoder();
    }

    public AbsoluteEncoder getAbsoluteEncoder(Type type) {
        return sparkMax.getAbsoluteEncoder(type);
    }

    public void burnFlash() {
        sparkMax.burnFlash();
    }

    public void setPIDReference(double setpoint, CANSparkMax.ControlType controlType) {
        sparkPID.setReference(setpoint, controlType);
        pidSetpoint = setpoint;
    }

    public void setPositionPIDWrappingEnabled(boolean enable) {
        sparkPID.setPositionPIDWrappingEnabled(enable);
    }

    public void setPositionPIDWrappingMinInput(double min) {
        sparkPID.setPositionPIDWrappingMinInput(min);
    }

    public void setPositionPIDWrappingMaxInput(double max) {
        sparkPID.setPositionPIDWrappingMaxInput(max);
    }

    public void setInverted(boolean invert) {
        sparkMax.setInverted(invert);
    }

    public void setPIDFeedbackDevice(RelativeEncoder encoder) {
        sparkPID.setFeedbackDevice(encoder);
    }

    public void setPIDFeedbackDevice(AbsoluteEncoder encoder) {
        sparkPID.setFeedbackDevice(encoder);
    }

    public void setP(double p) {
        sparkPID.setP(p);
    }

    public void setI(double i) {
        sparkPID.setP(i);
    }

    public void setD(double d) {
        sparkPID.setP(d);
    }

    public void setIdleMode(IdleMode idleMode) {
        sparkMax.setIdleMode(idleMode);
    }
 
}

