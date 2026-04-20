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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;

import frc.lib.mechanisms.Mechanism;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A power profiling utility used to estimate time-dependent energy draw from the robot's battery as
 * a function of subsystem and mechanism.
 */
public class PowerProfiler {

    public record MechanismRegistration(String key, Mechanism<?> mechanism) {}

    private final List<MechanismRegistration> mechanisms = new ArrayList<>();

    // Per loop current draw
    private double totalCurrentAmps = 0.0;
    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    // Per loop power draw
    private double totalPowerWatts = 0.0;
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    // Accumulated energy draw since boot
    private double totalEnergyJoules = 0.0;
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    private boolean isInitialized = false;
    private double lastTimestamp = 0.0;

    public void registerMechanism(String key, Mechanism<?> mechanism) {
        mechanisms.add(new MechanismRegistration(key, mechanism));
    }

    public void periodicAfterScheduler() {
        double loopTimeSeconds = getLoopTime();
        for (var reg : mechanisms) {
            double currentAmps = Math.abs(reg.mechanism().getSupplyCurrent().in(Amps));
            double appliedVolts = Math.abs(reg.mechanism().getAppliedVoltage().in(Volts));
            reportUsage(reg.key(), currentAmps, appliedVolts, loopTimeSeconds);
        }

        Logger.recordOutput("PowerProfiler/CurrentAmps", totalCurrentAmps);
        Logger.recordOutput("PowerProfiler/PowerWatts", totalPowerWatts);
        Logger.recordOutput(
                "PowerProfiler/TotalEnergyWattHours", energyToWattHours(totalEnergyJoules));

        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("PowerProfiler/CurrentAmps/" + entry.getKey(), entry.getValue());
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("PowerProfiler/PowerWatts/" + entry.getKey(), entry.getValue());
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput(
                    "PowerProfiler/TotalEnergyWattHours/" + entry.getKey(),
                    energyToWattHours(entry.getValue()));
        }
        resetLoopTotals();
    }

    // Record a mechanism update and tally new resulting robot and subsystem totals
    private void reportUsage(
            String key, double currentAmps, double appliedVolts, double loopTimeSeconds) {
        double powerWatts = currentAmps * appliedVolts;
        double energyJoules = powerWatts * loopTimeSeconds;

        // New robot totals
        totalCurrentAmps += currentAmps;
        totalPowerWatts += powerWatts;
        totalEnergyJoules += energyJoules;

        // New subsystem mechanism (e.g. shooter/hood) totals
        subsystemCurrents.merge(key, currentAmps, Double::sum);
        subsystemPowers.merge(key, powerWatts, Double::sum);
        subsystemEnergies.merge(key, energyJoules, Double::sum);

        // New subsystem (e.g. shooter) totals
        rollUpSubsystemTotals(key, currentAmps, powerWatts, energyJoules);
    }

    // Roll up the subsystem totals from the mechanism level
    private void rollUpSubsystemTotals(
            String key, double currentAmps, double powerWatts, double energyJoules) {
        String[] parts = key.split("/");
        if (parts.length < 2) return;

        String prefix = "";
        for (int i = 0; i < parts.length - 1; i++) {
            prefix = prefix.isEmpty() ? parts[i] : prefix.concat("/").concat(parts[i]);
            subsystemCurrents.merge(prefix, currentAmps, Double::sum);
            subsystemPowers.merge(prefix, powerWatts, Double::sum);
            subsystemEnergies.merge(prefix, energyJoules, Double::sum);
        }
    }

    // Approximate RIO loop time in seconds
    private double getLoopTime() {
        double dt;
        double now = Timer.getFPGATimestamp();
        if (!isInitialized) {
            dt = 0.02;
            isInitialized = true;
        } else {
            dt = now - lastTimestamp;
        }
        lastTimestamp = now;
        return dt;
    }

    // Reset loop totals but maintain accumulated values
    private void resetLoopTotals() {
        totalCurrentAmps = 0.0;
        totalPowerWatts = 0.0;
        subsystemCurrents.replaceAll((k, v) -> 0.0);
        subsystemPowers.replaceAll((k, v) -> 0.0);
    }

    private static double energyToWattHours(double energy) {
        return energy / 3600.0;
    }
}
