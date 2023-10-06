package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;

public class SwerveModule extends SubsystemBase {
    public final int moduleNumber;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;
    private SparkMaxAbsoluteEncoder m_angleEncoder;

    private final PIDController m_drivePIDController = new PIDController(kSwerve.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            kSwerve.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kSwerve.kMaxModuleAngularSpeedRadiansPerSecond,
                    kSwerve.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS,
            Constants.kSwerve.DRIVE_KV);

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        this.moduleNumber = moduleNumber;

        m_driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_angleEncoder = m_turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        configureDevices();
        resetEncoders();
        stopAll();

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveEncoder.getVelocity(), new Rotation2d(m_angleEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveEncoder.getPosition(), new Rotation2d(m_angleEncoder.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_angleEncoder.getPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_angleEncoder.getPosition(),
                state.angle.getRadians());

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turnMotor.setVoltage(turnOutput);
    }

    /**
     * Reset the motor encoders to zero
     */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(0);
    }

    public void stopAll() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    private void configureDevices() {
        // Drive motor configuration.
        // NEO Motor connected to SParkMax
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.clearFaults();
        if (m_driveMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
            SmartDashboard.putString("Drive Motor Idle Mode", "Error");
        }
        m_driveMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
        m_driveMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
        m_driveMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
        m_driveMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
        m_driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);

        m_driveEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
        m_driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
        m_driveEncoder.setPosition(0);

        // Angle motor configuration.
        // Neo Motor connected to SParkMax
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.clearFaults();
        if (m_turnMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
            SmartDashboard.putString("Turn Motor Idle Mode", "Error");
        }
        m_turnMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
        m_turnMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
        m_turnMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

        /**
         * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
         * input
         * Native will ready 0.0 -> 1.0 for each revolution.
         */
        m_angleEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
        m_angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
        m_angleEncoder.setInverted(false);
    }

    /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  public void updateSmartdashboard() {
    SmartDashboard.putNumber("Module " + moduleNumber + " Drive Encoder", m_driveEncoder.getPosition());
    SmartDashboard.putNumber("Module " + moduleNumber + " Turn Encoder", m_turnEncoder.getPosition());
    SmartDashboard.putNumber("Module " + moduleNumber + " Angle Encoder", m_angleEncoder.getPosition());
  }
}