package team9442.frc2024.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team9442.frc2024.constants.DrivetrainConstants;

public class SwerveModuleConfigurator {
    public void configureDriveMotor(
            CANSparkMax driveMotor, SimpleMotorFeedforward driveFeedforward) {
        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        driveFeedforward =
                new SimpleMotorFeedforward(
                        SwerveModuleConstants.kDriveKs,
                        SwerveModuleConstants.kDriveKv,
                        SwerveModuleConstants.kDriveKa);

        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(DrivetrainConstants.kDriveMotorInvert);
        driveMotor.setIdleMode(DrivetrainConstants.kDriveIdleMode);
        driveMotor.setOpenLoopRampRate(SwerveModuleConstants.kOpenLoopRamp);
        driveMotor.setClosedLoopRampRate(SwerveModuleConstants.kClosedLoopRamp);
        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);

        drivePID.setP(SwerveModuleConstants.kDriveKp);
        drivePID.setI(SwerveModuleConstants.kDriveKi);
        drivePID.setD(SwerveModuleConstants.kDriveKd);
        drivePID.setFF(SwerveModuleConstants.kDriveKf);

        driveEncoder.setPositionConversionFactor(DrivetrainConstants.kDriveRotationsMeters);
        driveEncoder.setVelocityConversionFactor(DrivetrainConstants.kDriveRpmMetersPerSecond);
        driveEncoder.setPosition(0);
    }

    public void configureAngleMotor(CANSparkMax angleMotor, CANcoder canCoder) {
        SparkMaxPIDController anglePID = angleMotor.getPIDController();
        RelativeEncoder angleEncoder = angleMotor.getEncoder();

        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(DrivetrainConstants.kAngleMotorInvert);
        angleMotor.setIdleMode(DrivetrainConstants.kAngleIdleMode);
        angleMotor.setSmartCurrentLimit(SwerveModuleConstants.kAngleCurrentLimit);

        anglePID.setP(SwerveModuleConstants.kAngleKp);
        anglePID.setI(SwerveModuleConstants.kAngleKi);
        anglePID.setD(SwerveModuleConstants.kAngleKd);
        anglePID.setFF(SwerveModuleConstants.kAngleKf);

        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        anglePID.setPositionPIDWrappingMinInput(0);

        angleEncoder.setPositionConversionFactor(DrivetrainConstants.kAngleRotationsRadians);
        angleEncoder.setVelocityConversionFactor(DrivetrainConstants.kAngleRPMRadPerSec);
        angleEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition().getValue()));
    }

    public void configureCanCoder(CANcoder canCoder) {
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

    public SwerveModuleConfigurator() {}
    ;
}
