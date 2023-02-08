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
    private final AutoManager autoManager;
    private final Drivetrain drivetrain = new Drivetrain();

    Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);
    //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        configureButtonBindings();

        autoManager = AutoManager.getInstance();

        drivetrain.setDefaultCommand(
            new RunCommand(() -> drivetrain.drive(
                MathUtil.applyDeadband(-driverController.getY(), 0.06),
                MathUtil.applyDeadband(-driverController.getX(), 0.06),
                MathUtil.applyDeadband(-driverController.getZ(), 0.06),
                true),
                drivetrain));

    }

    private void configureButtonBindings() {
        // Trigger for brake
        new JoystickButton(driverController, OIConstants.kJoystickTrigger)
            .whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

        // Hat controls 
        new POVButton(driverController, 90)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                -0.04, 
                0, 
                false), 
                drivetrain));
        new POVButton(driverController, 270)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                0.04, 
                0, 
                false), 
                drivetrain));
        new POVButton(driverController, 0)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0.04, 
                0, 
                0, 
                false), 
                drivetrain));
        new POVButton(driverController, 180)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                -0.04, 
                0, 
                0, 
                false), 
                drivetrain));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
