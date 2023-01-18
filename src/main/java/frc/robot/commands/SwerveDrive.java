package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
    private Drivetrain sub_drivetrain;
    private Supplier<Double> fXSpeed;
    private Supplier<Double> fYSpeed;
    private Supplier<Double> fRot;
    private Supplier<Boolean> bFieldRelative;

    // Suppliers = non static 
    public SwerveDrive(Drivetrain sub_drivetrain, Supplier<Double> fXSpeed, Supplier<Double> fYSpeed, Supplier<Double> fRot, Supplier<Boolean> bFieldRelative) {
        this.sub_drivetrain = sub_drivetrain;
        this.fXSpeed = fXSpeed;
        this.fYSpeed = fYSpeed;
        this.fRot = fRot;
        this.bFieldRelative = bFieldRelative;
        addRequirements(sub_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        sub_drivetrain.drive(fXSpeed.get(), fYSpeed.get(), fRot.get(), bFieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {
        sub_drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
