package team9442.frc2024.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import team9442.frc2024.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] modules;
    private final double maxSpeed;
    private final double maxRotation;
    private final Pigeon2 gyro;

    public Drivetrain(
            team9442.frc2024.util.SwerveModule[] modules,
            Pigeon2 gyro,
            SwerveDriveKinematics kinematics,
            double maxSpeedMetersPerSecond,
            double maxRotationRadsPerSecond) {
        this.modules = modules;
        this.swerveKinematics = kinematics;
        this.swerveOdometry = new SwerveDriveOdometry(kinematics, getYaw(), getPositions());
        this.maxSpeed = maxSpeedMetersPerSecond;
        this.maxRotation = maxRotationRadsPerSecond;
        this.gyro = gyro;
        zeroGyro();
    }

    public Command drive(
            DoubleSupplier forwardBackAxis,
            DoubleSupplier leftRightAxis,
            DoubleSupplier rotationAxis,
            boolean isFieldRelative,
            boolean isOpenLoop) {
        return run(() -> {
                    // Grabbing input from suppliers, -1 to 1 axis value
                    double forwardBack = forwardBackAxis.getAsDouble();
                    double leftRight = leftRightAxis.getAsDouble();
                    double rotation = rotationAxis.getAsDouble();

                    // Converting to m/s
                    forwardBack *= maxSpeed;
                    leftRight *= maxSpeed;
                    rotation *= maxRotation;

                    // Get desired module states.
                    ChassisSpeeds chassisSpeeds =
                            isFieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            forwardBack, leftRight, rotation, getYaw())
                                    : new ChassisSpeeds(forwardBack, leftRight, rotation);

                    SwerveModuleState[] states =
                            swerveKinematics.toSwerveModuleStates(chassisSpeeds);

                    // 2024 discretization BETA
                    ChassisSpeeds.discretize(chassisSpeeds, .02);

                    setModuleStates(states, isOpenLoop);
                })
                .withName("SwerveDriveCommand");
    }

    /** To be used by auto. Use the drive method during teleop. */
    public void setModuleStates(SwerveModuleState[] states) {
        setModuleStates(states, false);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        // Makes sure the module states don't exceed the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }

        return currentStates;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getPosition();
        }

        return currentStates;
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValue());
    }

    public Command zeroGyroCommand() {
        return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
    }

    private void zeroGyro() {
        gyro.setYaw(0);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        for (SwerveModule module : modules) {
            builder.addStringProperty(
                    String.format("Module %d", module.moduleNumber),
                    () -> {
                        SwerveModuleState state = module.getState();
                        return String.format(
                                "%6.2fm/s %6.3fdeg",
                                state.speedMetersPerSecond, state.angle.getDegrees());
                    },
                    null);

            builder.addDoubleProperty(
                    String.format("Cancoder %d", module.moduleNumber),
                    () -> module.getCanCoder(),
                    null);

            builder.addDoubleProperty(
                    String.format("Angle %d", module.moduleNumber),
                    () -> module.getAngle().getDegrees(),
                    null);
        }
    }
}
