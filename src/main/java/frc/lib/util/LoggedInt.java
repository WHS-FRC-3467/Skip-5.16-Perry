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

/** Helper that records an int to the logger only when it changes. */
public class LoggedInt {
    private final String key;
    private int last;
    private boolean hasLast = false;

    public LoggedInt(String key) {
        this.key = key;
    }

    /**
     * Log the value if it differs from the last logged value.
     *
     * @param value The value to log.
     */
    public void log(int value) {
        if (!hasLast) {
            last = value;
            hasLast = true;
            Logger.recordOutput(key, value);
            return;
        }
        if (value != last) {
            last = value;
            Logger.recordOutput(key, value);
        }
    }

    /** Force a log regardless of previous value. */
    public void force(int value) {
        last = value;
        hasLast = true;
        Logger.recordOutput(key, value);
    }

    /** Get last recorded value (may be null). */
    public Integer getLast() {
        return hasLast ? Integer.valueOf(last) : null;
    }
}
