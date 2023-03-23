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
import com.team2914.robot.subsystems.DriverController;
import com.team2914.robot.subsystems.OperatorController;

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

    private final DriverController driverController;
    private final OperatorController operatorController;

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        autoManager = AutoManager.getInstance();
        drivetrain = Drivetrain.getInstance();
        lift = Lift.getInstance();
        claw = Claw.getInstance();
        driverController = DriverController.getInstance();
        operatorController = OperatorController.getInstance();

        configureButtonBindings();
        drivetrain.setDefaultCommand(
            new RunCommand(() -> drivetrain.drive(
                MathUtil.applyDeadband(-driverController.getY(), 0.06),
                MathUtil.applyDeadband(-driverController.getX(), 0.06),
                MathUtil.applyDeadband(-driverController.getZ(), 0.06)),
                drivetrain));

        lift.setDefaultCommand(
            new RunCommand(() -> lift.moveArm(
                MathUtil.applyDeadband(-operatorController.getY(), 0.06) * 0.15, 
                MathUtil.applyDeadband(operatorController.getX(), 0.06) * 0.15),
                lift)
        );

    }

    private void configureButtonBindings() {
        // DRIVER CONTROLS
        driverController.configureButtons();
        
        // OPERATOR CONTROLS
        operatorController.configureButtons();
    }

    public Command getAutonomousCommand() {
        return autoManager.getAutonomousCommand();
    }
}
