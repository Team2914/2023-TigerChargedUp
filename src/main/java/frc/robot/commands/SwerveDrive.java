package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
    private Drivetrain sub_drivetrain;
    private Supplier<Double> fXSpeed;
    private Supplier<Double> fYSpeed;
    private Supplier<Double> fRot;

    // Suppliers = non static 
    public SwerveDrive(Drivetrain sub_drivetrain, Supplier<Double> fXSpeed, Supplier<Double> fYSpeed, Supplier<Double> fRot) {
        this.sub_drivetrain = sub_drivetrain;
        this.fXSpeed = fXSpeed;
        this.fYSpeed = fYSpeed;
        this.fRot = fRot;
        addRequirements(sub_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        sub_drivetrain.drive(MathUtil.applyDeadband(fXSpeed.get(), 0.05), 
                             MathUtil.applyDeadband(fYSpeed.get(), 0.05), 
                             MathUtil.applyDeadband(fRot.get(), 0.05));

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
