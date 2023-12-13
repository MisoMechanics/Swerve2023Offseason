package team9442.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public abstract class SparkmaxSubsystem implements PeriodicSubsystem {
    private final CANSparkMax master;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController controller;
    protected final ServoConstants constants;
    private final List<CANSparkMax> followers = new ArrayList<>();
    protected final double kDt;

    private PeriodicIO periodicIO = new PeriodicIO();

    public SparkmaxSubsystem(CANSparkMax master, ServoConstants constants, double kDt) {
        this.master = master;
        this.kDt = kDt;
        this.constants = constants;
        this.encoder = master.getEncoder();
        this.controller = master.getPIDController();
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;

        // Outputs
        public ControlType controlMode = ControlType.kDutyCycle;
        public double demand = 0;
        public double feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = encoder.getPosition();
        periodicIO.velocity = encoder.getVelocity();
        periodicIO.current = master.getOutputCurrent();
        periodicIO.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void writePeriodicOutputs() {
        controller.setReference(
                periodicIO.demand,
                periodicIO.controlMode,
                0,
                periodicIO.feedforward,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public void setOpenloop(double output) {
        periodicIO.controlMode = ControlType.kDutyCycle;
        periodicIO.demand = output;
        periodicIO.feedforward = 0;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kPosition;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kSmartMotion;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        periodicIO.controlMode = ControlType.kVelocity;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = velocity;
    }
    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(IdleMode.kCoast);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(IdleMode.kCoast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(IdleMode mode) {
        master.setIdleMode(mode);
        for (CANSparkMax follower : followers) {
            follower.setIdleMode(mode);
        }
    }

    /** Sets the selected sensor to default reset position from constants */
    public void resetEncoder() {
        setEncoder(constants.resetPosition);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    protected REVLibError setEncoder(double position) {
        return encoder.setPosition(position);
    }

    /**
     * @return the velocity in the output units
     */
    public double getVelocity() {
        return periodicIO.velocity;
    }

    /**
     * @return ths position in the output units
     */
    public double getPosition() {
        return periodicIO.position;
    }

    public boolean atPositionSetpoint(double tolerance) {
        return Math.abs(getPosition() - periodicIO.demand) < tolerance;
    }

    public boolean atPositionSetpoint() {
        return atPositionSetpoint(constants.positionTolerance);
    }

    public boolean atVelocitySetpoint(double tolerance) {
        return Math.abs(getVelocity() - periodicIO.demand) < tolerance;
    }

    public boolean atVelocitySetpoint() {
        return atVelocitySetpoint(constants.velocityTolerance);
    }

    /**
     * @return the timestamp for the position and velocity measurements
     */
    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getMasterCurrent() {
        return periodicIO.current;
    }

    protected REVLibError addFollower(CANSparkMax follower, boolean invertFromMaster) {
        final var error = follower.follow(master, invertFromMaster);
        if (REVLibError.kOk == error) {
            followers.add(follower);
        }
        return error;
    }

    /**
     * @param error
     * @return is fatal
     */
    public static boolean reportError(String action, REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }
        DriverStation.reportError(String.format("Failed %s: %s", action, error.toString()), false);
        return false;
    }
}
