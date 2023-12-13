package team9442.frc2024.constants;

import static team9442.lib.SparkmaxSubsystem.reportError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import team9442.frc2024.constants.GlobalConstants.ElevatorIds;
import team9442.lib.ServoConstants;

public class ElevatorConstants {

    public static final CANSparkMax kMaster =
            new CANSparkMax(ElevatorIds.kMasterId, MotorType.kBrushless);
    public static final CANSparkMax kFollower =
            new CANSparkMax(ElevatorIds.kFollowerId, MotorType.kBrushless);

    public static final double kGearboxReduction = 14.0 / 48.0 * 30.0 / 40.0 * 18.0 / 24.0;
    public static final double kDrumDiameter = Units.inchesToMeters(4);
    public static final double kDrumCircumferenceMeters = kDrumDiameter * Math.PI;
    public static final double kMotorRotationsToMeters =
            kDrumCircumferenceMeters * kGearboxReduction;
    public static final double kMotorRPMToMetersPerSec =
            kDrumCircumferenceMeters * kGearboxReduction / 60.0;

    public static final float kMinimumPosition = 0;
    public static final float kMaximumPosition = 3;
    public static final float kMidPosition = (kMinimumPosition + kMaximumPosition) / 2;

    public static final double kPositionToleranceMeters = 0.05;
    public static final double kVelocityToleranceMetersPerSec = 0.05;

    public static final double kSmartMotionMaxVelocity = 2;
    public static final double kSmartMotionMaxAcceleration = 2;

    public static final double kP = 1;
    public static final double kD = 1;

    public static final int kSmartMotionPIDSlot = 0;

    public static final double kHomingCurrent = 5;

    public static ServoConstants kElevatorServoConstants =
            new ServoConstants(
                    kVelocityToleranceMetersPerSec,
                    kPositionToleranceMeters,
                    kMinimumPosition,
                    kMaximumPosition,
                    kMinimumPosition,
                    kHomingCurrent);

    public static void config() {
        reportError("Resetting elevator master factory defaults", kMaster.restoreFactoryDefaults());
        reportError("Setting elevator master timeout", kMaster.setCANTimeout(1000));
        reportError(
                "Enabling elevator forward limit",
                kMaster.enableSoftLimit(SoftLimitDirection.kForward, true));
        reportError(
                "Enabling reverse limit",
                kMaster.enableSoftLimit(SoftLimitDirection.kReverse, true));
        reportError(
                "Setting elevator reverse soft limit",
                kMaster.setSoftLimit(SoftLimitDirection.kReverse, kMinimumPosition));
        reportError(
                "Setting elevator forward soft limit",
                kMaster.setSoftLimit(SoftLimitDirection.kForward, kMaximumPosition));

        final var encoder = kMaster.getEncoder();
        reportError(
                "Setting elevator encoder postition convertsion factor",
                encoder.setPositionConversionFactor(kMotorRotationsToMeters));
        reportError(
                "Setting elevator encoder velocity convertsion factor",
                encoder.setVelocityConversionFactor(kMotorRPMToMetersPerSec));

        final var controller = kMaster.getPIDController();

        reportError("Elevator set P", controller.setP(kP, kSmartMotionPIDSlot));
        reportError("Elevator set I", controller.setI(0, kSmartMotionPIDSlot));
        reportError("Elevator set D", controller.setP(kD, kSmartMotionPIDSlot));

        reportError(
                "Elevator set max velocity",
                controller.setSmartMotionMaxVelocity(kSmartMotionMaxVelocity, kSmartMotionPIDSlot));
        reportError(
                "Elevator set max acceleration",
                controller.setSmartMotionMaxAccel(
                        kSmartMotionMaxAcceleration, kSmartMotionPIDSlot));

        reportError("Elevator master burn flash", kMaster.burnFlash());
    }

    static {
        config();
    }

    private ElevatorConstants() {}
}
