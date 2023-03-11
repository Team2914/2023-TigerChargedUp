package com.team2914.robot.subsystems;

import com.team2914.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriverController extends TigerController {
    private static DriverController instance = null;

    private DriverController(int port) {
        super(port);
    }

    public static DriverController getInstance() {
        if (instance == null) {
            instance = new DriverController(OIConstants.DRIVER_CONTROLLER_PORT);
        }

        return instance;
    }

    @Override
    public void configureButtons() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        // Trigger for brake
        new JoystickButton(joystick, OIConstants.JOYSTICK_TRIGGER)
            .whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

        new JoystickButton(joystick, 8)
            .whileTrue(new RunCommand(() -> 
                drivetrain.setFieldRelative(!drivetrain.isFieldRelative())
            , drivetrain));

        // Hat controls 
        new POVButton(joystick, 90)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                -0.04, 
                0), 
                drivetrain));
        new POVButton(joystick, 270)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0, 
                0.04, 
                0), 
                drivetrain));
        new POVButton(joystick, 0)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                0.04, 
                0, 
                0), 
                drivetrain));
        new POVButton(joystick, 180)
            .whileTrue(new RunCommand(() -> drivetrain.drive(
                -0.04, 
                0, 
                0), 
                drivetrain));

    }
}
