package com.team2914.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.SwerveAutoBuilderEx;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerEx;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import com.team2914.robot.utils.MiscUtil;
import com.team2914.robot.subsystems.Claw;
import com.team2914.robot.subsystems.Drivetrain;
import com.team2914.robot.subsystems.Lift;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.team2914.robot.Constants.AutoConstants;
import com.team2914.robot.Constants.DriveConstants;
import com.team2914.robot.commands.CommandMoveLift;
import com.team2914.robot.commands.IntakeGamePiece;

/* AutoManager singleton. It manages autonomous opmodes. */
public class AutoManager {
    private static AutoManager instance = null;
    private HashMap<String, Command> eventMap;
    private SwerveAutoBuilderEx autoBuilder;
    private final SendableChooser<Command> autoChooser;
    private final Claw claw = Claw.getInstance();
    private final Lift lift = Lift.getInstance();

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
        autoChooser.addOption("mid cube yolo", AutoPlaceCube());
        autoChooser.addOption("Red charge station", RedCharge());
        autoChooser.addOption("Blue charge station", BlueCharge());
        autoChooser.addOption("Backwards - 1 meter", Backwards1Meter().withTimeout(1));
        autoChooser.addOption("Backwards - 3.5 meter", Backwards3AndAHalfMeter());
        
        SmartDashboard.putData("Autonomous OpMode", autoChooser);
    }

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }

        return instance;
    }

    private CommandBase AutoPlaceCube() {
        return new SequentialCommandGroup(
            new RunCommand(() -> claw.runIntake(), claw).withTimeout(0.2),
            new RunCommand(() -> claw.closeClaw(), claw).withTimeout(0.5),
            new RunCommand(() -> lift.setArmMid(), claw).withTimeout(3),
            new RunCommand(() -> claw.runOuttake(), claw).withTimeout(0.5)
        );
    }
    
    private CommandBase PIDTEST() {
        return autoBuilder.fullAuto(
            PathPlanner.loadPath("PIDTEST", new PathConstraints(
                                                        AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
                                                        AutoConstants.MAX_ACCEL_METERS_PER_SEC_SQUARED))
        );
    }

    private CommandBase RedCharge() {
        return autoBuilder.fullAuto(
            PathPlanner.loadPath("RedCharge", new PathConstraints(
                                                        AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
                                                        AutoConstants.MAX_ACCEL_METERS_PER_SEC_SQUARED))
        );
    }

    private CommandBase BlueCharge() {
        return autoBuilder.fullAuto(
            PathPlanner.loadPath("BlueCharge", new PathConstraints(
                                                        AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
                                                        AutoConstants.MAX_ACCEL_METERS_PER_SEC_SQUARED))
        );
    }

    private CommandBase Backwards1Meter() {
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        List<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(currentPose.getTranslation(), currentPose.getRotation(), currentPose.getRotation()));
        points.add(new PathPoint(new Translation2d(currentPose.getX() - 1.62, currentPose.getY()), currentPose.getRotation(), currentPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(
                2, 
                1), 
            false, 
            points);

        return new PPSwerveControllerCommand(
            traj, 
            Drivetrain.getInstance()::getPose, 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_X_CONSTANTS), 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_Y_CONSTANTS), 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_ROT_CONSTANTS), 
            Drivetrain.getInstance()::setModuleStates, 
            Drivetrain.getInstance());
    }

    private CommandBase Backwards3AndAHalfMeter() {
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        List<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(currentPose.getTranslation(), currentPose.getRotation(), currentPose.getRotation()));
        points.add(new PathPoint(new Translation2d(currentPose.getX() - 3.5, currentPose.getY()), currentPose.getRotation(), currentPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(
                2, 
                1), 
            false, 
            points);

        return new PPSwerveControllerCommand(
            traj, 
            Drivetrain.getInstance()::getPose, 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_X_CONSTANTS), 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_Y_CONSTANTS), 
            MiscUtil.pidControllerFromConstants(AutoConstants.PID_ROT_CONSTANTS), 
            Drivetrain.getInstance()::setModuleStates, 
            Drivetrain.getInstance());
    }
    
    private HashMap<String, Command> buildEventMap() {
        return new HashMap<>(Map.ofEntries(
            Map.entry("runIntake", new IntakeGamePiece(claw, lift))
        ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
