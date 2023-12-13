package team9442.lib;

public class ServoConstants {
    public final double velocityTolerance;
    public final double positionTolerance;

    public final double minPosition;
    public final double maxPosition;
    public final double resetPosition;

    public final double homingCurrent;

    public ServoConstants(
            double velocityTolerance,
            double positionTolerance,
            double minPosition,
            double maxPosition,
            double resetPosition,
            double homingCurrent) {
        this.velocityTolerance = velocityTolerance;
        this.positionTolerance = positionTolerance;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.resetPosition = resetPosition;
        this.homingCurrent = homingCurrent;
    }
}
