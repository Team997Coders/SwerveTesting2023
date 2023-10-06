// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain(m_gyro);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final Command m_resetEncoders = Commands.runOnce(m_robotDrive::reset_encoders);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driverController.a().onTrue(m_resetEncoders);

    m_robotDrive.modules[0].updateSmartdashboard();
    m_robotDrive.modules[1].updateSmartdashboard();
    m_robotDrive.modules[2].updateSmartdashboard();
    m_robotDrive.modules[3].updateSmartdashboard();

    SmartDashboard.putNumber("Gyro Yaw angle:", m_gyro.getYaw());


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -m_driverController.getRawAxis(Constants.OperatorConstants.TRANSLATION_Y_AXIS),
                -m_driverController.getRawAxis(Constants.OperatorConstants.TRANSLATION_X_AXIS),
                -m_driverController.getRawAxis(Constants.OperatorConstants.ROTATION_AXIS),
                true),
            m_robotDrive));

    SmartDashboard.putData("Reset Encoders",
        Commands.run(() -> m_robotDrive.reset_encoders()));
        
  }
}
