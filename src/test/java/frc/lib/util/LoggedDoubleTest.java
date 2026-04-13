/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link LoggedDouble} to lock in "log only on change" semantics. */
public class LoggedDoubleTest {
    @BeforeEach
    void setup() {
        // Initialize HAL as other tests do; fail the test run if initialization fails.
        HAL.initialize(500, 0);
    }

    @Test
    void firstLogSetsLast() {
        LoggedDouble l = new LoggedDouble("Test/First");
        // Initially no last value
        // Implementation uses -999.0 as the sentinel for "no last"; verify sentinel.
        assertEquals(Double.valueOf(-999.0), l.getLast());

        l.log(5.0);
        assertEquals(Double.valueOf(5.0), l.getLast());
    }

    @Test
    void tolerancePreventsSmallChanges() {
        // tolerance of 0.5: changes smaller than or equal to 0.5 should not update last
        LoggedDouble l = new LoggedDouble("Test/Tolerance", 0.5);

        // First log sets the last
        l.log(1.0);
        assertEquals(Double.valueOf(1.0), l.getLast());

        // Small change within tolerance should not update
        l.log(1.25); // delta 0.25 < 0.5
        assertEquals(Double.valueOf(1.0), l.getLast());

        // Larger change outside tolerance should update
        l.log(1.6); // delta 0.6 > 0.5
        assertEquals(Double.valueOf(1.6), l.getLast());
    }

    @Test
    void nanLoggingUsesSentinelAndDoesNotThrow() {
        LoggedDouble l = new LoggedDouble("Test/NaN");

        // Ensure hasLast is true so NaN handling path is exercised
        l.log(1.0);
        assertEquals(Double.valueOf(1.0), l.getLast());

        // Logging NaN should be converted to sentinel (-99999.0) and update last
        l.log(Double.NaN);
        assertEquals(Double.valueOf(-99999.0), l.getLast());
    }

    @Test
    void repeatedSameValueDoesNotChangeLast() {
        LoggedDouble l = new LoggedDouble("Test/Repeat");
        l.log(10);
        assertEquals(Double.valueOf(10.0), l.getLast());

        // Logging the same value again should leave last unchanged
        l.log(10);
        assertEquals(Double.valueOf(10.0), l.getLast());
    }

    @Test
    void changeAfterRepeatsUpdatesLast() {
        LoggedDouble l = new LoggedDouble("Test/Change");
        l.log(1);
        l.log(1);
        assertEquals(Double.valueOf(1.0), l.getLast());

        l.log(2);
        assertEquals(Double.valueOf(2.0), l.getLast());
    }

    @Test
    void forceAlwaysSetsLast() {
        LoggedDouble l = new LoggedDouble("Test/Force");

        // Force without prior log should set the value
        l.force(7);
        assertEquals(Double.valueOf(7.0), l.getLast());

        // Force should update even if same value (semantics: force sets and records)
        l.force(7);
        assertEquals(Double.valueOf(7.0), l.getLast());
    }
}
