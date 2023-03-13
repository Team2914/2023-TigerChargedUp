package com.team2914.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2914.lib.gyro.BNO055;
import com.team2914.lib.gyro.BNO055.opmode_t;
import com.team2914.lib.gyro.BNO055.vector_type_t;
import com.team2914.robot.subsystems.Drivetrain;

public class CommandBalanceRobot extends CommandBase {
    private final Drivetrain drivetrain;
    private boolean doBalance = true;
    private double pitch;

    public CommandBalanceRobot(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        BNO055 gyro = BNO055.getInstance(opmode_t.OPERATION_MODE_IMUPLUS, vector_type_t.VECTOR_EULER);
        pitch = gyro.getVector()[2];
        doBalance = (Math.abs(pitch) >= 10) ? true : false;
    }

    @Override
    public void execute() {
        if (doBalance) drivetrain.drive(Math.sin(Math.toRadians(pitch - (Math.signum(pitch)*10))) * -1, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !doBalance;
    }
    
}
