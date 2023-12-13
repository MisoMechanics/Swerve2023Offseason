package team9442.frc2024.constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team9442.frc2024.constants.GlobalConstants.DrivetrainIds;
import team9442.frc2024.util.SwerveModule;
import team9442.frc2024.util.SwerveModuleConstants;

/**
 * This class contains values that remain constant while the robot is running.
 *
 * <p>It's split into categories using subclasses, preventing too many members from being defined on
 * one class.
 */
public class DrivetrainConstants {
    /** All joystick, button, and axis IDs. */
    public static final double kAxisDeadzone = 0.1;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter kXDriveLimiter = new SlewRateLimiter(4);
    public static final SlewRateLimiter kYDriveLimiter = new SlewRateLimiter(4);
    public static final SlewRateLimiter kThetaDriveLimiter = new SlewRateLimiter(4);

    /** All swerve constants. */

    /** Hardware intialization */
    public static final Pigeon2 kGyro = new Pigeon2(DrivetrainIds.kGyroId);
    public static final CANSparkMax kMod0Drive =
            new CANSparkMax(DrivetrainIds.kMod0DriveId, MotorType.kBrushless);
    public static final CANSparkMax kMod0Angle =
            new CANSparkMax(DrivetrainIds.kMod0AngleId, MotorType.kBrushless);
    public static final CANcoder kMod0Encoder =
            new CANcoder(DrivetrainIds.kMod0EncoderId);

    public static final CANSparkMax kMod1Drive =
            new CANSparkMax(DrivetrainIds.kMod1DriveId, MotorType.kBrushless);
    public static final CANSparkMax kMod1Angle =
            new CANSparkMax(DrivetrainIds.kMod1AngleId, MotorType.kBrushless);
    public static final CANcoder kMod1Encoder =
            new CANcoder(DrivetrainIds.kMod1EncoderId);

    public static final CANSparkMax kMod2Drive =
            new CANSparkMax(DrivetrainIds.kMod2DriveId, MotorType.kBrushless);
    public static final CANSparkMax kMod2Angle =
            new CANSparkMax(DrivetrainIds.kMod2AngleId, MotorType.kBrushless);
    public static final CANcoder kMod2Encoder =
            new CANcoder(DrivetrainIds.kMod2EncoderId);

    public static final CANSparkMax kMod3Drive =
            new CANSparkMax(DrivetrainIds.kMod3DriveId, MotorType.kBrushless);
    public static final CANSparkMax kMod3Angle =
            new CANSparkMax(DrivetrainIds.kMod3AngleId, MotorType.kBrushless);
    public static final CANcoder kMod3Encoder =
            new CANcoder(DrivetrainIds.kMod3EncoderId);
    
    /**
     * Module specific constants. CanCoder offset is in DEGREES, not radians like the rest of
     * the repo. This is to make offset slightly more accurate and easier to measure.
     */
    public static final SwerveModuleConstants kMod0Constants =
            new SwerveModuleConstants(1, 2, 3, 203.115234);

    public static final SwerveModuleConstants kMod1Constants =
            new SwerveModuleConstants(4, 5, 6, 191.074219);

    public static final SwerveModuleConstants kMod2Constants =
            new SwerveModuleConstants(7, 8, 9, 203.906250);

    public static final SwerveModuleConstants kMod3Constants =
            new SwerveModuleConstants(10, 11, 12, 155.214844);

    public static final SwerveModule[] kModules =
            new SwerveModule[] {
                new SwerveModule(0, kMod0Constants, kMod0Drive, kMod0Angle, kMod0Encoder),
                new SwerveModule(1, kMod1Constants, kMod1Drive, kMod1Angle, kMod1Encoder),
                new SwerveModule(2, kMod2Constants, kMod2Drive, kMod2Angle, kMod2Encoder),
                new SwerveModule(3, kMod3Constants, kMod3Drive, kMod3Angle, kMod3Encoder),
            };

    /** Constants that apply to the whole drive train. */
    public static final double kTrackWidth =
            Units.inchesToMeters(
                    18.75); // Width of the drivetrain measured from the middle of the wheels.
    public static final double kWheelBase =
            Units.inchesToMeters(
                    18.75); // Length of the drivetrain measured from the middle of the wheels.
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    public static final double kDriveGearRatio = 6.75 / 1.0; // 6.75:1
    public static final double kDriveRotationsMeters = kWheelCircumference / kDriveGearRatio;
    public static final double kDriveRpmMetersPerSecond = kDriveRotationsMeters / 60.0;
    public static final double kAngleGearRatio = 12.8 / 1.0; // 12.8:1
    public static final double kAngleRotationsRadians = (Math.PI * 2) / kAngleGearRatio;
    public static final double kAngleRPMRadPerSec = kDriveRotationsMeters / 60.0;

    /** Swerve constraints. */
    public static final double kMaxVelocityMetersPerSecond = 3.0;
    public static final double kMaxAngularRadiansPerSecond = 4.0;

    /** Inversions. */
    public static final boolean kCancoderCCWPos = false;
    public static final boolean kDriveMotorInvert = true;
    public static final boolean kAngleMotorInvert = false;

    /** Idle modes. */
    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kAngleIdleMode = IdleMode.kCoast;

    /** PID Values. */
    public static final double kAutoXKP = 1.0;
    public static final double kAutoYKP = 1.0;
    public static final double kAutoThetaKP = 1.0;

    /** Constraints. */
    public static final double kAutoMaxVelMPS = 2.0;
    public static final double kAutoMaxAccelMPS2 = 5.0;

    private DrivetrainConstants() {};
}

