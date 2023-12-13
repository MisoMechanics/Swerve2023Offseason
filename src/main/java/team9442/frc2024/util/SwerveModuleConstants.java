package team9442.frc2024.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import team9442.frc2024.constants.GlobalConstants.DrivetrainIds;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int canCoderID;
    public final double canCoderOffsetDegrees;

    /** Speed ramp. */
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    /** Current limiting. */
    public static final int kDriveCurrentLimit = 35;
    public static final int kAngleCurrentLimit = 25;

    /** Drive motor PID values. */
    public static final double kDriveKp = 0.1;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;
    public static final double kDriveKf = 0.0;

    /** Drive motor characterization. */
    public static final double kDriveKs = 0.11937;
    public static final double kDriveKv = 2.6335;
    public static final double kDriveKa = 0.46034;

    /** Angle motor PID values. */
    public static final double kAngleKp = 1.5;
    public static final double kAngleKi = 0.0;
    public static final double kAngleKd = 0.1;
    public static final double kAngleKf = 0.0;

    public SwerveModuleConstants(
            int driveMotorID, int angleMotorID, int canCoderID, double canCoderOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}