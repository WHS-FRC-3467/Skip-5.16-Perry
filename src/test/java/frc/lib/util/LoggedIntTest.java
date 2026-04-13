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
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link LoggedInt} to lock in "log only on change" semantics. */
class LoggedIntTest {

    @BeforeEach
    void setup() {
        // Initialize HAL as other tests do; fail the test run if initialization fails.
        HAL.initialize(500, 0);
    }

    @Test
    void firstLogSetsLast() {
        LoggedInt l = new LoggedInt("Test/First");
        // Initially no last value
        assertNull(l.getLast());

        l.log(5);
        assertEquals(Integer.valueOf(5), l.getLast());
    }

    @Test
    void repeatedSameValueDoesNotChangeLast() {
        LoggedInt l = new LoggedInt("Test/Repeat");
        l.log(10);
        assertEquals(Integer.valueOf(10), l.getLast());

        // Logging the same value again should leave last unchanged
        l.log(10);
        assertEquals(Integer.valueOf(10), l.getLast());
    }

    @Test
    void changeAfterRepeatsUpdatesLast() {
        LoggedInt l = new LoggedInt("Test/Change");
        l.log(1);
        l.log(1);
        assertEquals(Integer.valueOf(1), l.getLast());

        l.log(2);
        assertEquals(Integer.valueOf(2), l.getLast());
    }

    @Test
    void forceAlwaysSetsLast() {
        LoggedInt l = new LoggedInt("Test/Force");

        // Force without prior log should set the value
        l.force(7);
        assertEquals(Integer.valueOf(7), l.getLast());

        // Force should update even if same value (semantics: force sets and records)
        l.force(7);
        assertEquals(Integer.valueOf(7), l.getLast());
    }
}
