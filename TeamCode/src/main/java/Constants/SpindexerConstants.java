package Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class SpindexerConstants {
    private SpindexerConstants() {}

    // CR servo power for indexing rotation (positive = advance direction)
    public static double SPIN_POWER = 0.3;

    // Number of slots in the spindexer
    public static final int SLOT_COUNT = 4;

    // Linkage (flipper) servo positions
    public static final double LINKAGE_DOWN = 0.67;
    public static final double LINKAGE_UP = 0;

    // Flipper state machine timing (ms)
    public static long FIRE_TIME_MS = 175;
    public static long RETRACT_TIME_MS = 150;

    // Debounce/cooldown after flipper retract before next advance (ms)
    public static long DEBOUNCE_MS = 250;

    // Settle times (retained for command sequencing)
    public static long ROTATION_SETTLE_MS = 125;
    public static long ROTATION_LONG_SETTLE_MS = 380;
}
