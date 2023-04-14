package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SwerveModuleConstants;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {

  public static final double ARM_MAX_DEG = -1.2;
  public static final double ARM_MIN_DEG = -107.4;
  public static final double ARM_MAX_EXT = 999;
  public static final double ARM_MIN_EXT = -999;
  /** All joystick, button, and axis IDs. */
  public static class kControls {
    public static final double AXIS_DEADZONE = 0.1;

    public static final int DRIVE_JOYSTICK_ID = 0;
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */

    public static final double TRACK_WIDTH = Units.inchesToMeters(16.5); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double NEO_MAX_SPEED = 5676;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final double DRIVE_GEAR_RATIO = 8.14 / 1.0; // 8.14:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = (DRIVE_ROTATIONS_TO_METERS / 60.0);
    // public static final double DRIVE_RPM_TO_METERS_PER_SECOND = (NEO_MAX_SPEED / 60.0) * DRIVE_ROTATIONS_TO_METERS;
    public static final double ANGLE_GEAR_RATIO = 12.8 / 1.0; // 12.8:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    // public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_RPM_TO_METERS_PER_SECOND / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = false;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants(
      5,
      6,
      0,
      // 203.115234
      15.205
      //20.3
    );

    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants(
      7,
      8,
      1,
      // 191.074219
      239.677734375
      //238.711
    );

    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants(
      9,
      10,
      2,
      // 203.906250
      120.234375 //121.113 deg
    );

    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants(
      11,
      12,
      3,
      //181.845
      // 155.214844
      184.834
    );
  }

  public static class kAuto {
    /** PID Values. */
    public static final double X_CONTROLLER_KP = 1.0;
    public static final double Y_CONTROLLER_KP = 1.0;
    public static final double THETA_CONTROLLER_KP = 1.0;
    
    /** Constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5.0;
  }
  // ARM 
  public static class ArmConstants {
    public static final boolean isTunable = true;

    public static final double MIN_SETPOINT = 1.05; //intake
    public static final double MAX_SETPOINT = 0.77;  //start
    public static final double SCORING_SETPOINT = 0.92;
    public static final double ARM_KP = 2;
    public static final double ARM_KI = 0.15;
    public static final double ARM_KD = 0.5;

    public static final double minAngle = -62.0;
    public static final double maxAngle = 40.0;
}

// EXTENDER
public static class ExtenderConstants {
    public static final double EXTENDER_MIN_EXTENSION = 0.0;
    public static final double EXTENDER_MAX_EXTENSION = 155.0; 

    public static final boolean isTunable = true;

}

// GRIPPER
public static class GripperConstants {

    public static final boolean isTunable = false;

    public static final double fullOpenWhenExtended = -1.0;
    public static final double fullOpen = 14.0;
    public static final double fullClosed = 50.0;

    public static final double closeCone = 50.0;
    public static final double closeCube = 30.0;
}

public static class IntakeConstants {

  public static final boolean isTunable = true;

  public static final double DROOL_SPEED = -0.2;
  public static final double SPIT_SPEED = 0.5;
  public static final double SPEW_SPEED = 0.8; // 0.6
  public static final double SLURP_SPEED = 0.15;
  public static final double AUTO_DROOL_SPEED = -0.2;
  public static final double AUTO_SPIT_SPEED = 0.5;
  public static final double AUTO_SPEW_SPEED = 0.8; // 0.6
  public static final double AUTO_SLURP_SPEED = 0.15;
}

public static class MarvinConstants {
  public static final double MARVIN_SPEED = 2000;
}
}
