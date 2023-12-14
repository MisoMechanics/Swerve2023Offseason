package team9442.frc2024.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public final int moduleNumber;

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;
    private final SimpleMotorFeedforward driveFeedforward;

    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;

    private final CANcoder canCoder;
    private final double maxSpeed;

    private double lastAngle;

    private SwerveModuleConfigurator configurator = new SwerveModuleConfigurator();

    public SwerveModule(
            int moduleNumber,
            CANSparkMax driveMotor,
            CANSparkMax angleMotor,
            CANcoder canCoder,
            double maxSpeed) {
        this.moduleNumber = moduleNumber;

        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        driveFeedforward = null;

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;
        this.maxSpeed = maxSpeed;

        configurator.configureDriveMotor(driveMotor, driveFeedforward);
        configurator.configureAngleMotor(angleMotor, canCoder);
        configurator.configureCanCoder(canCoder);

        lastAngle = getState().angle.getRadians();
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        // Prevents angle motor from turning further than it needs to.
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = SwerveModuleState.optimize(state, getState().angle);

        if (isOpenLoop) {
            double speed = state.speedMetersPerSecond / maxSpeed;
            drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
        } else {
            drivePID.setReference(
                    state.speedMetersPerSecond,
                    CANSparkMax.ControlType.kVelocity,
                    0,
                    driveFeedforward.calculate(state.speedMetersPerSecond));
        }

        double angle =
                Math.abs(state.speedMetersPerSecond) <= maxSpeed * 0.01
                        ? lastAngle
                        : state.angle.getRadians();

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, rot);
    }

    public Double getCanCoder() {
        return canCoder.getAbsolutePosition().getValue() * 360;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(angleEncoder.getPosition());
    }

    public SwerveModulePosition getPosition() {
        double distance = driveEncoder.getPosition();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }
}
