package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;
  
    private final SwerveDriveOdometry swerveOdometry;
  
    private final AHRS gyro;
  
    public Swerve() {
      gyro = new AHRS();
  
      modules = new SwerveModule[] {
        new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
        new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
        new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
        new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
      };
  
