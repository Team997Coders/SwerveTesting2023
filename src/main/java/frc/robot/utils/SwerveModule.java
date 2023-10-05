package frc.robot.utils;

import java.io.PrintStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        this.moduleNumber = moduleNumber;
        System.out.println("Module " + moduleNumber);

        System.out.println("Drive Motor " + constants.driveMotorID);
        m_driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);

        System.out.println("Angle Motor " + constants.angleMotorID);
        m_turnMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_angleEncoder = m_turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        configureDevices();
        resetEncoders();
        stopAll();
    }

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
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.clearFaults();
        if (m_turnMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
            SmartDashboard.putString("Turn Motor Idle Mode", "Error");
        }
        m_turnMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
        m_turnMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
        m_turnMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

        m_angleEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
        m_angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
        m_angleEncoder.setInverted(false);
    }
}