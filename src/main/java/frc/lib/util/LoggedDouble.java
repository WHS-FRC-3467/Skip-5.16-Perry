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

import org.littletonrobotics.junction.Logger;

/** Helper that records a double to the logger only when it changes beyond an absolute tolerance. */
public class LoggedDouble {
    private final String key;
    private final double tolerance;
    private double last;
    private boolean hasLast = false;

    /** No tolerance (exact change required). */
    public LoggedDouble(String key) {
        this(key, 0.0);
    }

    /**
     * Absolute tolerance only. Value is logged when |value - last| > tolerance.
     *
     * @param key logging key
     * @param tolerance minimum absolute delta required to trigger a log
     */
    public LoggedDouble(String key, double tolerance) {
        this.key = key;
        this.tolerance = Math.abs(tolerance);
    }

    /** Log the value if it differs from the last logged value by more than the tolerance. */
    public void log(double value) {
        if (!hasLast) {
            last = value;
            hasLast = true;
            Logger.recordOutput(key, value);
            return;
        }

        // Sentinel value to prevent strange behavior from transitions from Nan -> finite number
        if (Double.isNaN(value)) {
            value = -99999.0;
        }
        double delta = Math.abs(value - last);
        if (delta > tolerance) {
            last = value;
            Logger.recordOutput(key, value);
        }
    }

    /** Force a log regardless of previous value. */
    public void force(double value) {
        last = value;
        hasLast = true;
        Logger.recordOutput(key, value);
    }

    /** Get last recorded value (returns -999 if null). */
    public Double getLast() {
        return hasLast ? Double.valueOf(last) : -999.0;
    }
}
