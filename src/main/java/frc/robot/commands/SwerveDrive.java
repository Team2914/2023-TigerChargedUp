package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
    private Drivetrain sub_drivetrain;

    public SwerveDrive(Drivetrain sub_drivetrain) {
        this.sub_drivetrain = sub_drivetrain;

        addRequirements(sub_drivetrain);
    }
    
}
