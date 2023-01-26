// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.*;

// The container for the robot. Contains subsystems, OI devices, and commands.
public class RobotContainer {
  
  private final Drivetrain sub_drivetrain;
  private final Vision sub_vision;
  private final Joystick con_driver;
  
  public RobotContainer() {
    sub_drivetrain = new Drivetrain();
    sub_vision = new Vision();
    con_driver = new Joystick(OIConstants.kDriverControllerPort); 
                                                    
    sub_drivetrain.setDefaultCommand(new SwerveDrive(sub_drivetrain, 
                                                    () -> con_driver.getX(), 
                                                    () -> con_driver.getY(), 
                                                    () -> con_driver.getZ()));
    
    configureBindings();
  }

  private void configureBindings() {
    /* Driver controller */
    // Trigger toggles field relative driving
    new JoystickButton(con_driver, OIConstants.kJoystickTrigger)
        .onTrue(new InstantCommand(() -> sub_drivetrain.setFieldRelative(!sub_drivetrain.isFieldRelative())));

    /* Operator controller */
  }

  // Use this to pass the autonomous command to the main Robot class.
  public Command getAutonomousCommand() {
    return null;
  }
}
