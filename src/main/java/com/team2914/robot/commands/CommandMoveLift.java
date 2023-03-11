package com.team2914.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2914.robot.subsystems.Lift;

public class CommandMoveLift extends CommandBase {
    private final Lift lift;
    private double pos;

    public CommandMoveLift(double pos, Lift lift) {
        this.lift = lift;
        this.pos = pos;

        addRequirements(lift);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //lift.setMotorPosition(pos);
    }

}
