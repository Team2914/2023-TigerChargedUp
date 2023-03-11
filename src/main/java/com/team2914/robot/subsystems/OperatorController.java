package com.team2914.robot.subsystems;

import com.team2914.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class OperatorController extends TigerController {
    private static OperatorController instance = null;

    private OperatorController(int port) {
        super(port);
    }

    public static OperatorController getInstance() {
        if (instance == null) {
            instance = new OperatorController(OIConstants.OPERATOR_CONTROLLER_PORT);
        }

        return instance;
    }

    @Override
    public void configureButtons() {
        Claw claw = Claw.getInstance();
        Lift lift = Lift.getInstance();

        new JoystickButton(joystick, 1)
            .whileTrue(new RunCommand(() -> claw.setIntakeSpeed(0.75), claw))
            .whileFalse(new RunCommand(() -> claw.setIntakeSpeed(0), claw));
            new JoystickButton(joystick, 2)
            .whileTrue(new RunCommand(() -> claw.setIntakeSpeed(-0.75), claw))
            .whileFalse(new RunCommand(() -> claw.setIntakeSpeed(0), claw));
        
        new JoystickButton(joystick, 3)
            .whileTrue(new RunCommand(() -> claw.closeClaw(), claw));
            

        new JoystickButton(joystick, 5)
            .whileTrue(new RunCommand(() -> claw.openClaw(), claw))
            .whileFalse(new RunCommand(claw.closed ? () -> {} : () -> claw.set(0), claw));

        new JoystickButton(joystick, 4)
            .whileTrue(new RunCommand(() -> lift.setArmHigh(), lift));

        new JoystickButton(joystick, 6)
            .whileTrue(new RunCommand(() -> lift.setArmMid(), lift));

        new POVButton(joystick, 0)
            .whileTrue(new RunCommand(() -> lift.setArmLow(), lift));

        new POVButton(joystick, 180)
            .whileTrue(new RunCommand(() -> lift.liftGamePiece(), lift));
    }
}
