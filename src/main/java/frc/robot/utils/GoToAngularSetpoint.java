package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoToAngularSetpoint extends CommandBase {
    private double setPointRad;
    private static Drivetrain drivetrain;
    private SwerveModule[] modules;
    private double[] angles;
    private PIDController frontLeftAngularPID;
    private PIDController frontRightAngularPID;
    private PIDController backRightAngularPID;
    private PIDController backLeftAngularPID;

    public GoToAngularSetpoint(Drivetrain drivetrain, double[] angles) {
        this.setPointRad = setPointRad;
        this.drivetrain = drivetrain;
        this.modules = drivetrain.getModules();
        this.angles = angles;
        this.frontLeftAngularPID = new PIDController(modules[0].constants.angleMotorKP, 0, 0);
        this.frontRightAngularPID = new PIDController(modules[1].constants.angleMotorKP, 0, 0);
        this.backRightAngularPID = new PIDController(modules[2].constants.angleMotorKP, 0, 0);
        this.backLeftAngularPID = new PIDController(modules[3].constants.angleMotorKP, 0, 0);
    }

    @Override
    public void execute(){
        drivetrain.setAngleMotorSpeeds(
        frontLeftAngularPID.calculate(modules[0].getPosition().angle.getRadians(), angles[0]),
        frontLeftAngularPID.calculate(modules[1].getPosition().angle.getRadians(), angles[0]),
        frontLeftAngularPID.calculate(modules[2].getPosition().angle.getRadians(), angles[0]),
        frontLeftAngularPID.calculate(modules[3].getPosition().angle.getRadians(), angles[0])
        );
    }

    
}
