// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_ANGULAR_OFFSET = 0;//-Math.PI / 2;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_ANGULAR_OFFSET = 0;//Math.PI;
    public static final double BACK_RIGHT_ANGULAR_OFFSET = Math.PI;//Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 6;
    public static final int REAR_LEFT_DRIVE_CAN_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 8;
    public static final int REAR_RIGHT_DRIVE_CAN_ID = 4;

    public static final int FRONT_LEFT_TURN_CAN_ID = 7;
    public static final int REAR_LEFT_TURN_CAN_ID = 3;
    public static final int FRONT_RIGHT_TURN_CAN_ID = 9;
    public static final int REAR_RIGHT_TURN_CAN_ID = 5;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    public static final double DRIVING_P = 0.05;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_AMPS_LIMIT = 50; 
    public static final int TURNING_MOTOR_AMPS_LIMIT = 20; 
  }

  public static final class AutoConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 4;
    public static final double MAX_ACCEL_METERS_PER_SEC_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADS_PER_SEC_SQUARED = Math.PI;

    public static PIDConstants PID_X_CONSTANTS = new PIDConstants(2.22, 0, 0);
    public static PIDConstants PID_Y_CONSTANTS = new PIDConstants(2.22, 0, 0);
    public static PIDConstants PID_ROT_CONSTANTS = new PIDConstants(5, 0, 0);

    public static Translation2d BLU_GRID1_BB_TOP_LEFT = new Translation2d(1.40, 1.85);
    public static Translation2d BLU_GRID1_BB_BTM_RITE = new Translation2d(2.5, 0);
    public static Translation2d BLU_GRID2_BB_TOP_LEFT = new Translation2d(1.40, 3.55);
    public static Translation2d BLU_GRID2_BB_BTM_RITE = new Translation2d(2.5, 1.85);
    public static Translation2d BLU_GRID3_BB_TOP_LEFT = new Translation2d(1.40, 5.5);
    public static Translation2d BLU_GRID3_BB_BTM_RITE = new Translation2d(2.5, 3.55);
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;

    public static final int JOYSTICK_TRIGGER = 1;

    public static final String PHOTON_CAMERA_NAME = "HD_Pro_Webcam_C920";
  }

  public static final class VisionConstants {
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0)
    );
  }

  public static final class LiftConstants {
    public static final int SHOULDER_MOTOR_CAN_ID = 13;
    public static final int ELBOW_MOTOR_CAN_ID = 11;
    public static final int SHOULDER_FOLLOW_MOTOR_CAN_ID = 12;
    public static final int ELBOW_FOLLOW_MOTOR_CAN_ID = 10;
    public static final PIDConstants SHOULDER_PID = new PIDConstants(0.0000006, 0.008, 0.008);
    public static final PIDConstants ELBOW_PID = new PIDConstants(0.0000006, 0.008, 0.008);
    public static final double LIFT_MIN_OUTPUT = -1;
    public static final double LIFT_MAX_OUTPUT = 1;
    public static final double SHOULDER_LENGTH = 200;
    public static final double ELBOW_LENGTH = 200;
    public static final double SHOULDER_GEAR_RATIO = 64.0 / 1.0;
    public static final double SHOULDER_SPROCKET_RATIO = 32.0 / 28.0;
    public static final double ELBOW_GEAR_RATIO = 36.0 / 1;
    public static final double ELBOW_SPROCKET_RATIO = 1.0;
  }

  public static final class ClawConstants {
    public static final int ROTATE_CAN_ID = 15;
    public static final int INTAKE_CAN_ID = 14;
  }
}
