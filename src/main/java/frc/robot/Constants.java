// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
/*
 * 
 * Swerve Drive configuration:
 * 6/7   --F--  8/1
 *  |            |
 *  L            R
 *  |            |
 * 4/5   --R--  2/3
 * 
 */
public final class Constants {
  public static final class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.0); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(24.0); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    // SDS Mk4i L2 Drive Ratio = 6.75:1, Angle Ratio = 150/7:1
    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_MOTOR_GEAR_RATIO = 12.8 / 1.0; // 12.8:1

    /* note that these angle constant refer to the CTRE absolute encoder */
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2);
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

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
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPModuleTurningController = 1;
    public static final double kPModuleDriveController = 1;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = false;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    // Front Left Module
    public static final SwerveModuleConstants kMOD_1_Constants = new SwerveModuleConstants(
      6,
      7,
      254.5 // 360 * 0.7069
    );

    // Front Right
    public static final SwerveModuleConstants kMOD_2_Constants = new SwerveModuleConstants(
      4,
      5,
      152.0 // 360*0.4221 
    );

    // Back Left
    public static final SwerveModuleConstants kMOD_3_Constants = new SwerveModuleConstants(
      2,
      3,
      0.0
    );

    // Back Right
    public static final SwerveModuleConstants kMOD_4_Constants = new SwerveModuleConstants(
      1,
      8,
      0.0
    );
  }

  public static final class OperatorConstants {
    public static final int TRANSLATION_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int TRANSLATION_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    public static final int GYRO_RESET_BUTTON = XboxController.Button.kY.value;

    public static final int kDriverControllerPort = 0;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
