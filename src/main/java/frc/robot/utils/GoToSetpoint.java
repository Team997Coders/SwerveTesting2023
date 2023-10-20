package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class GoToSetpoint extends CommandBase {
    private double setPointRad;
    private static SwerveModule swervemodule;
    private  final PIDController anglePIDController;

    public GoToSetpoint(double setPointRad, SwerveModule swerveModule) {
        this.setPointRad = setPointRad;
        this.swervemodule = swerveModule;
        anglePIDController = new PIDController(swerveModule.constants.angleMotorKP, 0, 0);
    }

    
}
