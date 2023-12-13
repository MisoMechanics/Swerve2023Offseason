package team9442.frc2024.constants;

public final class GlobalConstants {
    public static final double kDt = 0.02;

    public static final class ElevatorIds {
        public static final int kMasterId = 10;
        public static final int kFollowerId = 11;

        private ElevatorIds() {}
    }

    public static final class DrivetrainIds {
        public static final int kGyroId = 0;

        public static final int kMod0DriveId = 1;
        public static final int kMod0AngleId = 2;
        public static final int kMod0EncoderId = 3;

        public static final int kMod1DriveId = 4;
        public static final int kMod1AngleId = 5;
        public static final int kMod1EncoderId = 6;
        
        public static final int kMod2DriveId = 7;
        public static final int kMod2AngleId = 8;
        public static final int kMod2EncoderId = 9;

        public static final int kMod3DriveId = 10;
        public static final int kMod3AngleId = 11;
        public static final int kMod3EncoderId = 12;

        private DrivetrainIds() {}
    }

    private GlobalConstants() {}
}
