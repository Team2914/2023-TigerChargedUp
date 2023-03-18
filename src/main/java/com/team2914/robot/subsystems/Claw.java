package com.team2914.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2914.robot.Constants.AutoConstants;
import com.team2914.robot.Constants.ClawConstants;
import com.team2914.robot.Constants.VisionConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static Claw instance = null;
    private final CANSparkMax rotateMotor;
    private final CANSparkMax intakeMotor;

    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private Color detectedColor;
    private ColorMatchResult colorMatchResult;
    private int distance;
    public boolean closed = false;

    private Claw() {
        intakeMotor = new CANSparkMax(ClawConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);

        rotateMotor = new CANSparkMax(ClawConstants.ROTATE_CAN_ID, MotorType.kBrushed);
        rotateMotor.setIdleMode(IdleMode.kBrake);
        rotateMotor.setSmartCurrentLimit(8);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(VisionConstants.CONE_COLOR);
        colorMatcher.addColorMatch(VisionConstants.CUBE_COLOR);
        colorMatcher.addColorMatch(VisionConstants.DEFAULT_COLOR);
    }

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("/Claw/Claw closed", closed);
        SmartDashboard.putNumber("/Claw/Claw rotation current", rotateMotor.getOutputCurrent());
        SmartDashboard.putNumber("/Claw/Intake rotation voltage", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("/Claw/Intake rotation current", intakeMotor.getOutputCurrent());

        distance = colorSensor.getProximity();
        SmartDashboard.putNumber("/Claw/Distance from object", distance);

        detectedColor = colorSensor.getColor();
        SmartDashboard.putNumber("/Claw/Color R", detectedColor.red);
        SmartDashboard.putNumber("/Claw/Color G", detectedColor.green);
        SmartDashboard.putNumber("/Claw/Color B", detectedColor.blue);

        colorMatchResult = colorMatcher.matchClosestColor(detectedColor);

        if (colorMatchResult.color.equals(VisionConstants.CONE_COLOR) && distance >= 80) {
            SmartDashboard.putString("/Claw/Detected game piece", "Cone");
        } else if (colorMatchResult.color.equals(VisionConstants.CUBE_COLOR) && distance >= 80) {
            SmartDashboard.putString("/Claw/Detected game piece", "Cube");
        } else {
            SmartDashboard.putString("/Claw/Detected game piece", "None");
        }
    }

    public void closeClaw() {
        set(-0.15);
        closed = true;
    }

    public void set(double val) {
        rotateMotor.set(val);
    }

    public void openClaw() {
        set(0.3);
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
