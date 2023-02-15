package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class CommandMoveLift extends CommandBase {
    private final Lift lift;

    public CommandMoveLift(Lift lift) {
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        lift.setLiftPosition(Math.PI / 4.0);
    }
}
