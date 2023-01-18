// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

// The container for the robot. Contains subsystems, OI devices, and commands.
public class RobotContainer {
  
  private final Drivetrain sub_drivetrain;
  private final XboxController con_driver;
  
  public RobotContainer() {
    sub_drivetrain = new Drivetrain();
    con_driver = new XboxController(OIConstants.kDriverControllerPort);

    sub_drivetrain.setDefaultCommand(new SwerveDrive(sub_drivetrain,
                                                    () -> con_driver.getRawAxis(OIConstants.kLeftStickXAxis),
                                                    () -> con_driver.getRawAxis(OIConstants.kLeftStickYAxis),
                                                    () -> con_driver.getRawAxis(OIConstants.kRightStickXAxis),
                                                    () -> con_driver.getAButton()));   

    configureBindings();
  }

  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
