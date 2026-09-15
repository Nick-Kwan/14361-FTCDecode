package Constants;

public final class EnumConstants {

    public enum AllianceColor {
        Red, Blue
    }

    public enum FlickState {
        Idle, Start, Extended, Retracted
    }

    public enum SpindexerState {
        IDLE, SPINNING, ALIGNED, FLIPPING, RETRACTING, COOLDOWN
    }

    public enum IntakeState {
        Idle, Intaking, Reversing
    }

    public enum LimelightMode {
        GoalTracking, TagTracking
    }

    public enum ShootMode {
        Sorted, Unsorted
    }
}
