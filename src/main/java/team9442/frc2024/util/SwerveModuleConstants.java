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

public class SwerveModuleConstants {
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

    private SwerveModuleConstants(){

    }
}
