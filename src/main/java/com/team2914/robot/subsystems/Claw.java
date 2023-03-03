package com.team2914.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2914.robot.Constants.ClawConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static Claw instance = null;
    private final TalonSRX rotateTalon;
    private final TalonSRX intakeTalon;

    private Claw() {
        rotateTalon = new TalonSRX(ClawConstants.ROTATE_TALON_CHANNEL);
        intakeTalon = new TalonSRX(ClawConstants.INTAKE_TALON_CHANNEL);

        rotateTalon.setNeutralMode(NeutralMode.Brake);
        intakeTalon.setNeutralMode(NeutralMode.Coast);
    }

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }

    public void setIntakeSpeed(double speed) {
        intakeTalon.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setRotateSpeed(double speed) {
        rotateTalon.set(TalonSRXControlMode.PercentOutput, speed);
    }

}
