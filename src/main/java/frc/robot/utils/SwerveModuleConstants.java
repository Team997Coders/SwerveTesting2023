package frc.robot.utils;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final double angleEncoderOffsetDegrees;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleEncoderOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleEncoderOffsetDegrees = angleEncoderOffsetDegrees;
    }
}
