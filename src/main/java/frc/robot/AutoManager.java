package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/* AutoManager singleton. It manages autonomous opmodes. */
public class AutoManager {
    private static AutoManager instance = null;
    private HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;

    private AutoManager() {
        eventMap = buildEventMap();
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose, 
            drivetrain::resetPose,
            DriveConstants.kDriveKinematics, 
            //new PIDConstants(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), 
            //new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD), 
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0),
            drivetrain::setModuleStates, 
            eventMap, 
            drivetrain);

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("PIDTEST", PIDTEST());
        
        SmartDashboard.putData("Autonomous OpMode", autoChooser);
    }

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }

        return instance;
    }
    
    private CommandBase PIDTEST() {
        return autoBuilder.fullAuto(
            PathPlanner.loadPath("PIDTEST", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared))
        );
    }

    private HashMap<String, Command> buildEventMap() {
        return new HashMap<>(Map.ofEntries(
            Map.entry("event1", Commands.print("balls")),
            Map.entry("event2", Commands.print("another balls"))
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
