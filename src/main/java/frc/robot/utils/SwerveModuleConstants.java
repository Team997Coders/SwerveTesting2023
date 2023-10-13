package frc.robot.utils;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final Boolean driveMotorIsInverted;
    public final Boolean angleMotorIsInverted;
    public final Boolean angleEncoderIsInverted;
    public final double angleEncoderOffsetRadians;
    public final Boolean driveEncoderIsInverted;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleEncoderOffsetRadians, Boolean driveMotorIsInverted, Boolean angleMotorIsInverted, Boolean angleEncoderIsInverted, Boolean driveEncoderIsInverted) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleEncoderOffsetRadians = angleEncoderOffsetRadians;
        this.driveMotorIsInverted = driveMotorIsInverted;
        this.angleMotorIsInverted = angleMotorIsInverted;
        this.angleEncoderIsInverted = angleEncoderIsInverted; 
        this.driveEncoderIsInverted = driveEncoderIsInverted;
    }
}
