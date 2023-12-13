package team9442.frc2024.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import team9442.frc2024.constants.DrivetrainConstants;

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
    private final double canCoderOffsetDegrees;

    private double lastAngle;

    private SwerveModuleConstants swerveModuleConstants;

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants, CANSparkMax driveMotor, CANSparkMax angleMotor, CANcoder canCoder) {
        this.moduleNumber = moduleNumber;

        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        driveFeedforward =
                new SimpleMotorFeedforward(
                        constants.kDriveKs,
                        constants.kDriveKv,
                        constants.kDriveKa);

        angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        canCoder = new CANcoder(constants.canCoderID);
        canCoderOffsetDegrees = constants.canCoderOffsetDegrees;

        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        configureDriveMotor(driveMotor);
        configureAngleMotor(angleMotor);
        configureCanCoder(canCoder);

        lastAngle = getState().angle.getRadians();

        swerveModuleConstants = constants;
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        // Prevents angle motor from turning further than it needs to.
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = SwerveModuleState.optimize(state, getState().angle);

        if (isOpenLoop) {
            double speed = state.speedMetersPerSecond / DrivetrainConstants.kMaxVelocityMetersPerSecond;
            drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
        } else {
            drivePID.setReference(
                    state.speedMetersPerSecond,
                    CANSparkMax.ControlType.kVelocity,
                    0,
                    driveFeedforward.calculate(state.speedMetersPerSecond));
        }

        double angle =
                Math.abs(state.speedMetersPerSecond)
                                <= DrivetrainConstants.kMaxVelocityMetersPerSecond * 0.01
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

    private void configureDriveMotor(CANSparkMax driveMotor){
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(DrivetrainConstants.kDriveMotorInvert);
        driveMotor.setIdleMode(DrivetrainConstants.kDriveIdleMode);
        driveMotor.setOpenLoopRampRate(swerveModuleConstants.kOpenLoopRamp);
        driveMotor.setClosedLoopRampRate(swerveModuleConstants.kClosedLoopRamp);
        driveMotor.setSmartCurrentLimit(swerveModuleConstants.kDriveCurrentLimit);

        drivePID.setP(swerveModuleConstants.kDriveKp);
        drivePID.setI(swerveModuleConstants.kDriveKi);
        drivePID.setD(swerveModuleConstants.kDriveKd);
        drivePID.setFF(swerveModuleConstants.kDriveKf);

        driveEncoder.setPositionConversionFactor(DrivetrainConstants.kDriveRotationsMeters);
        driveEncoder.setVelocityConversionFactor(DrivetrainConstants.kDriveRpmMetersPerSecond);
        driveEncoder.setPosition(0);

    }

    private void configureAngleMotor(CANSparkMax driveMotor){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(DrivetrainConstants.kAngleMotorInvert);
        angleMotor.setIdleMode(DrivetrainConstants.kAngleIdleMode);
        angleMotor.setSmartCurrentLimit(swerveModuleConstants.kAngleCurrentLimit);

        anglePID.setP(swerveModuleConstants.kAngleKp);
        anglePID.setI(swerveModuleConstants.kAngleKi);
        anglePID.setD(swerveModuleConstants.kAngleKd);
        anglePID.setFF(swerveModuleConstants.kAngleKf);

        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        anglePID.setPositionPIDWrappingMinInput(0);

        angleEncoder.setPositionConversionFactor(DrivetrainConstants.kAngleRotationsRadians);
        angleEncoder.setVelocityConversionFactor(DrivetrainConstants.kAngleRPMRadPerSec);
        angleEncoder.setPosition(
                Units.degreesToRadians(
                        canCoder.getAbsolutePosition().getValue() - canCoderOffsetDegrees));
    }

    private void configureCanCoder(CANcoder canCoder) {
        CANcoderConfigurator configurator = canCoder.getConfigurator();
        MagnetSensorConfigs canCoderConfiguration = new MagnetSensorConfigs();
        canCoderConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        if (DrivetrainConstants.kCancoderCCWPos) {
            canCoderConfiguration.withSensorDirection(
                    SensorDirectionValue.CounterClockwise_Positive);
        } else {
            canCoderConfiguration.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        }
        // canCoderConfiguration.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond; RESOLVED IN PHOENIX 6

        configurator.apply(new CANcoderConfiguration());
        configurator.apply(canCoderConfiguration);
    }
}
