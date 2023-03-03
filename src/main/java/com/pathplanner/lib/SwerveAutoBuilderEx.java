package com.pathplanner.lib;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SwerveAutoBuilderEx extends BaseAutoBuilder {
  private final SwerveDriveKinematics kinematics;
  private final PIDConstants translationXConstants;
  private final PIDConstants translationYConstants;
  private final PIDConstants rotationConstants;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final Subsystem[] driveRequirements;

  private final boolean useKinematics;

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationXConstants PID Constants for the controller that will correct for translation
   *     error in the X axis
   * @param translationYConstants PID Constants for the controller that will correct for translation
   *     error in the Y axis
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public SwerveAutoBuilderEx(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      PIDConstants translationXConstants,
      PIDConstants translationYConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        translationXConstants,
        translationYConstants,
        rotationConstants,
        outputChassisSpeeds,
        eventMap,
        false,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationXConstants PID Constants for the controller that will correct for translation
   *     error in the X axis
   * @param translationYConstants PID Constants for the controller that will correct for translation
   *     error in the Y axis
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public SwerveAutoBuilderEx(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      SwerveDriveKinematics kinematics,
      PIDConstants translationXConstants,
      PIDConstants translationYConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        kinematics,
        translationXConstants,
        translationYConstants,
        rotationConstants,
        outputModuleStates,
        eventMap,
        false,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationXConstants PID Constants for the controller that will correct for translation
   *     error in the X axis
   * @param translationYConstants PID Constants for the controller that will correct for translation
   *     error in the Y axis
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public SwerveAutoBuilderEx(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      PIDConstants translationXConstants,
      PIDConstants translationYConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = null;
    this.translationXConstants = translationXConstants;
    this.translationYConstants = translationYConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = null;
    this.outputChassisSpeeds = outputChassisSpeeds;
    this.driveRequirements = driveRequirements;

    this.useKinematics = false;
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationXConstants PID Constants for the controller that will correct for translation
   *     error in the X axis
   * @param translationYConstants PID Constants for the controller that will correct for translation
   *     error in the Y axis
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public SwerveAutoBuilderEx(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      SwerveDriveKinematics kinematics,
      PIDConstants translationXConstants,
      PIDConstants translationYConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = kinematics;
    this.translationXConstants = translationXConstants;
    this.translationYConstants = translationYConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.driveRequirements = driveRequirements;

    this.useKinematics = true;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    if (useKinematics) {
      return new PPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          kinematics,
          pidControllerFromConstants(translationXConstants),
          pidControllerFromConstants(translationYConstants),
          pidControllerFromConstants(rotationConstants),
          outputModuleStates,
          useAllianceColor,
          driveRequirements);
    } else {
      return new PPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          pidControllerFromConstants(translationXConstants),
          pidControllerFromConstants(translationYConstants),
          pidControllerFromConstants(rotationConstants),
          outputChassisSpeeds,
          useAllianceColor,
          driveRequirements);
    }
  }
}
