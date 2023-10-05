package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class Drivetrain extends SubsystemBase {

  private AHRS m_gyro;

  private final SwerveModule[] modules = new SwerveModule[] {
      new SwerveModule(1, Constants.kSwerve.kMOD_1_Constants), // Front Left
      new SwerveModule(2, Constants.kSwerve.kMOD_2_Constants), // Front Right
      new SwerveModule(3, Constants.kSwerve.kMOD_3_Constants), // Back Left
      new SwerveModule(4, Constants.kSwerve.kMOD_4_Constants), // Back Right
  };

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kSwerve.WHEEL_BASE / 2.0, Constants.kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(Constants.kSwerve.WHEEL_BASE / 2.0, -Constants.kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(-Constants.kSwerve.WHEEL_BASE / 2.0, Constants.kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(-Constants.kSwerve.WHEEL_BASE / 2.0, -Constants.kSwerve.TRACK_WIDTH / 2.0));

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          modules[0].getPosition(), // front left
          modules[1].getPosition(), // front right
          modules[2].getPosition(), // back left
          modules[3].getPosition() // back right
      });

  public Drivetrain() {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      m_gyro = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    zeroGyro();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw());
  }

  private void zeroGyro() {
    m_gyro.zeroYaw();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

      ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

      SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states);
  }

  private void setModuleStates(SwerveModuleState[] states) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber]);
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  /* a couple of utility commands */
  public Command resetYaw() {
    return runOnce(
        () -> {
          m_gyro.reset();
        });
  }

  public Command reset_encoders() {
    return runOnce(
        () -> {
          modules[0].resetEncoders();
          modules[1].resetEncoders();
          modules[2].resetEncoders();
          modules[3].resetEncoders();
        });
  }
}