package com.team2914.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2914.robot.subsystems.Lift;

public class CommandMoveLift extends CommandBase {
    private final Lift lift;
    private final int level;

    public CommandMoveLift(Lift lift, int level) {
        this.lift = lift;
        this.level = level;

        addRequirements(lift);
    }

    @Override 
    public void execute() {
        lift.setArmMid();
    }

    @Override
    public boolean isFinished() {
        //return lift.atSetpoint();
        return false;
    }
}
