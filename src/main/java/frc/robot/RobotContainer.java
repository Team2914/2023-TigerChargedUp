// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    private final Drivetrain sub_drivetrain = new Drivetrain();

    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        configureButtonBindings();

        sub_drivetrain.setDefaultCommand(
            new RunCommand(() -> sub_drivetrain.drive(
                MathUtil.applyDeadband(-m_driverController.getY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getZ(), 0.06),
                true),
                sub_drivetrain));

    }

    private void configureButtonBindings() {
        new JoystickButton(m_driverController, OIConstants.kJoystickTrigger)
            .whileTrue(new RunCommand(() -> sub_drivetrain.setX(), sub_drivetrain));
        new POVButton(m_driverController, 90)
            .whileTrue(new RunCommand(() -> sub_drivetrain.drive(
                0, 
                -0.04, 
                0, 
                false), 
                sub_drivetrain));
        new POVButton(m_driverController, 270)
            .whileTrue(new RunCommand(() -> sub_drivetrain.drive(
                0, 
                0.04, 
                0, 
                false), 
                sub_drivetrain));
        new POVButton(m_driverController, 0)
            .whileTrue(new RunCommand(() -> sub_drivetrain.drive(
                0.04, 
                0, 
                0, 
                false), 
                sub_drivetrain));
        new POVButton(m_driverController, 180)
            .whileTrue(new RunCommand(() -> sub_drivetrain.drive(
                -0.04, 
                0, 
                0, 
                false), 
                sub_drivetrain));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
