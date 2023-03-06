// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot;

import edu.wpi.first.math.MathUtil;
import com.team2914.robot.Constants.OIConstants;
import com.team2914.robot.commands.CommandMoveLift;
import com.team2914.robot.subsystems.Drivetrain;
import com.team2914.robot.subsystems.Lift;
import com.team2914.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
    private final AutoManager autoManager;
    private final Drivetrain drivetrain;
    private final Lift lift;
    private final Claw claw;

    private final Joystick driverController;
    private final Joystick operatorController;
    boolean isFieldRelativePressed = false;

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        autoManager = AutoManager.getInstance();
        drivetrain = Drivetrain.getInstance();
        lift = Lift.getInstance();
        claw = Claw.getInstance();

        driverController = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);
        operatorController = new Joystick(OIConstants.OPERATOR_CONTROLLER_PORT);

        
        configureButtonBindings();
        drivetrain.setDefaultCommand(
            new RunCommand(() -> drivetrain.drive(
                MathUtil.applyDeadband(-driverController.getY(), 0.06),
                MathUtil.applyDeadband(-driverController.getX(), 0.06),
                MathUtil.applyDeadband(-driverController.getZ(), 0.06)),
                drivetrain));

        /*lift.setDefaultCommand(
            new RunCommand(() -> lift.moveArm(
                MathUtil.applyDeadband(-operatorController.getY(), 0.06),
                MathUtil.applyDeadband(-operatorController.getX(), 0.06)),
                lift));*/

    }

    private void configureButtonBindings() {
        // DRIVER CONTROLS
        // Trigger for brake
        new JoystickButton(driverController, OIConstants.JOYSTICK_TRIGGER)
            .whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

        new JoystickButton(driverController, 8)
            .whileTrue(new RunCommand(() -> {
                if (!isFieldRelativePressed) {
                    isFieldRelativePressed = true;
                    drivetrain.setFieldRelative(!drivetrain.isFieldRelative());
                } else if (isFieldRelativePressed) {
                    isFieldRelativePressed = false;
                }
                
            }, drivetrain));

        // Hat controls 
        new POVButton(driverController, 90)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                -0.04, 
                0), 
                drivetrain));
        new POVButton(driverController, 270)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                0.04, 
                0), 
                drivetrain));
        new POVButton(driverController, 0)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0.04, 
                0, 
                0), 
                drivetrain));
        new POVButton(driverController, 180)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                -0.04, 
                0, 
                0), 
                drivetrain));

        // OPERATOR CONTROLS
        /*new JoystickButton(operatorController, 1)
            .whileTrue(new RunCommand(() -> claw.setIntakeSpeed(0.5), claw))
            .whileFalse(new RunCommand(() -> claw.setIntakeSpeed(0), claw));*/
            new JoystickButton(operatorController, 1)
            .whileTrue(new RunCommand(() -> lift.setMotorPosition(0.5), lift));

    }

    public Command getAutonomousCommand() {
        return autoManager.getAutonomousCommand();
    }
}
