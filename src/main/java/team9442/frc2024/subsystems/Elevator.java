package team9442.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import team9442.lib.ServoConstants;
import team9442.lib.SparkmaxSubsystem;

public class Elevator extends SparkmaxSubsystem {

    private double storedPosition = 0;

    public Elevator(
            CANSparkMax master, CANSparkMax follower, ServoConstants constants, double kDt) {
        super(master, constants, kDt);
        addFollower(follower, false);
    }

    private void setLength(double meters) {
        setPositionMotionMagic(meters, 0);
    }

    public Command length(double meters) {
        return length(() -> meters).until(this::atPositionSetpoint);
    }

    public Command length(DoubleSupplier meters) {
        return run(() -> setLength(meters.getAsDouble()));
    }

    public Command openloop(DoubleSupplier demand) {
        return run(() -> setOpenloop(demand.getAsDouble()));
    }

    private Command storePosition() {
        return Commands.runOnce(() -> storedPosition = getPosition());
    }

    public Command holdAtCall() {
        return Commands.sequence(storePosition(), length(storedPosition));
    }

    public Command home(double homingSpeed) {
        final var currentSpike =
                new Trigger(() -> getMasterCurrent() > constants.homingCurrent).debounce(0.4);
        return Commands.sequence(length(constants.resetPosition), openloop(() -> homingSpeed))
                .raceWith(Commands.waitSeconds(0.5).andThen(Commands.waitUntil(currentSpike)))
                .andThen(runOnce(this::resetEncoder));
    }
}
