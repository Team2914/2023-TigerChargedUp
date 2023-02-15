package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder motorEncoder;
    private final SparkMaxPIDController motorPIDController;

    public Lift() {
        motor = new CANSparkMax(LiftConstants.LIFT_MOTOR_CAN_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(2 * Math.PI);
        motorPIDController = motor.getPIDController();
        motorPIDController.setFeedbackDevice(motorEncoder);
        motorPIDController.setP(LiftConstants.LIFT_P);
        motorPIDController.setI(LiftConstants.LIFT_I);
        motorPIDController.setD(LiftConstants.LIFT_D);
        motorPIDController.setOutputRange(LiftConstants.LIFT_MIN_OUTPUT, LiftConstants.LIFT_MAX_OUTPUT);

        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift encoder position", motorEncoder.getPosition());
    }

    public void setLiftPosition(double position) {
        motorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void resetEncoders() {
        motorEncoder.setPosition(0);
    }
}
