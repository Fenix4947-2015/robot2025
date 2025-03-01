package frc.robot;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.time.temporal.Temporal;

public class Utils {

    private Utils() {
    }

    public static boolean elapsedAtLeastSince(long durationElapsedMs, Temporal since) {
        return Duration.between(since, Instant.now())
                .compareTo(Duration.of(durationElapsedMs, ChronoUnit.MILLIS)) >= 0;
    }
}
