package com.team2914.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.SwerveAutoBuilderEx;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.team2914.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.team2914.robot.Constants.AutoConstants;
import com.team2914.robot.Constants.DriveConstants;

/* AutoManager singleton. It manages autonomous opmodes. */
public class AutoManager {
    private static AutoManager instance = null;
    private HashMap<String, Command> eventMap;
    private SwerveAutoBuilderEx autoBuilder;
    private final SendableChooser<Command> autoChooser;

    private AutoManager() {
        eventMap = buildEventMap();
        Drivetrain drivetrain = Drivetrain.getInstance();
        autoBuilder = new SwerveAutoBuilderEx(
            drivetrain::getPose, 
            drivetrain::resetPose,
            DriveConstants.DRIVE_KINEMATICS, 
            AutoConstants.PID_X_CONSTANTS,
            AutoConstants.PID_Y_CONSTANTS,
            AutoConstants.PID_ROT_CONSTANTS,
            drivetrain::setModuleStates, 
            eventMap, 
            false,
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
            PathPlanner.loadPath("PIDTEST", new PathConstraints(
                                                        AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
                                                        AutoConstants.MAX_ACCEL_METERS_PER_SEC_SQUARED))
        );
    }

    private HashMap<String, Command> buildEventMap() {
        return new HashMap<>(Map.ofEntries(
            Map.entry("extendHigh", Commands.print("balls")),
            Map.entry("dropCone", Commands.print("another balls"))
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
