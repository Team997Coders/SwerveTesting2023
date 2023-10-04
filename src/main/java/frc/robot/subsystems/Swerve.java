package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;
  
    //private final SwerveDriveOdometry swerveOdometry;
  
    private final AHRS gyro;
  
    public Swerve() {
      gyro = new AHRS();
      gyro.reset();
  
      modules = new SwerveModule[] {
        new SwerveModule(0, Constants.kSwerve.kMOD_1_Constants),
        new SwerveModule(1, Constants.kSwerve.kMOD_2_Constants),
        new SwerveModule(2, Constants.kSwerve.kMOD_3_Constants),
        new SwerveModule(3, Constants.kSwerve.kMOD_4_Constants),
      };
    }

    public Command reset_encoders() {
        return runOnce(
            () -> {
                modules[0].resetEncoders();
                modules[1].resetEncoders();
                modules[2].resetEncoders();
                modules[3].resetEncoders();
            }
        );
    }
}