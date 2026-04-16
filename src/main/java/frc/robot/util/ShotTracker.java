// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.FlywheelConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

public class ShotTracker {

    private final ShooterSuperstructure shooter;
    private final RobotState robotState = RobotState.getInstance();

    // Fuel counts
    private @Getter double totalFuelCount = 0;

    // Linear velocity drop required to assume a shot (game piece(s)) may have passed through the
    // shooter, default
    // tuned from auto replay logs. Typically 1.0 - 2.0 m/s. With the drum shooter, an active shot
    // drop
    // is approximately at parity with an inactive shot drop because of the large MOI.
    private final LoggedTunableNumber shotDetectionThresholdMPS =
            new LoggedTunableNumber("ShotTracker/ShotDetectionThresholdMPS", 0.75);

    // Power draw required to assume a shot has passed through shooter
    private final LoggedTunableNumber powerDetectionThresholdWatts =
            new LoggedTunableNumber("ShotTracker/PowerDetectionThresholdWatts", 105);

    /**
     * Trigger determining whether a shot is believed to have potentially passed through the shooter
     * based on flywheel velocity drop from current setpoint, currently only registering true while
     * the robot is in a static scoring position to prevent false positives while moving
     */
    public final LoggedTrigger ballTrigger =
            new LoggedTrigger(
                    "ShotTracker/BallTrigger",
                    () ->
                            detectFlywheelDrop(
                                    MetersPerSecond.of(shotDetectionThresholdMPS.getAsDouble())));

    // Boolean describing whether a power drop event associated with the most recent potential shot
    // occurred. If no significant power drop was associated with the potential shot, assume no
    // ball(s)
    // passed through the shooter
    private boolean powerSink = false;

    // Heuristic for whether a shot has passed through the shooter in ~ 0.5s using flywheel velocity
    // drop and power spike
    private final Debouncer hopperEmptyDebouncer = new Debouncer(0.9, DebounceType.kRising);

    /**
     * Trigger input for hopperEmpty trigger, made explicit for compartmentalization and
     * tuning/debugging
     */
    public final LoggedTrigger hopperEmptyInput;

    private final LoggedTrigger hopperEmpty;

    public ShotTracker(ShooterSuperstructure shooter) {
        this.shooter = shooter;

        hopperEmptyInput =
                new LoggedTrigger(
                        "ShotTracker/hopperEmptyInput",
                        () -> this.shooter.staticShotState.getAsBoolean() && !powerSink);

        hopperEmpty =
                new LoggedTrigger(
                        "ShotTracker/hopperEmpty",
                        () -> hopperEmptyDebouncer.calculate(hopperEmptyInput.getAsBoolean()));
        robotState.setHopperEmpty(hopperEmpty);
        attachBallTriggers();
    }

    private void attachBallTriggers() {
        ballTrigger.onTrue(
                Commands.runOnce(
                        () -> {
                            if (this.shooter.getFlywheelPowerDraw().in(Watts)
                                    > powerDetectionThresholdWatts.getAsDouble()) {
                                powerSink = true;
                            } else {
                                powerSink = false;
                            }
                            if (hopperEmpty.negate().getAsBoolean())
                                totalFuelCount = totalFuelCount + 2.6;
                            Logger.recordOutput("ShotTracker/powerSink", powerSink);
                            Logger.recordOutput("ShotTracker/TotalFuelCount", totalFuelCount);
                        }));
    }

    /**
     * Determines whether the flywheel (drum) linear speed has dropped by at least the specified
     * speed from the current flywheel speed setpoint. Currently only factored to trigger while in
     * scoring position. Primarily for use in autos.
     *
     * <p>Gating the check behind having the flywheel be above a certain minimum velocity and the
     * static shot state helps prevent false positives from spurious velocity drops when the
     * flywheel is at low speed or when spinning up the flywheel in preparation for a shot.
     *
     * @param drop the magnitude of drop to compare
     */
    private boolean detectFlywheelDrop(LinearVelocity drop) {
        LinearVelocity desiredLinearVelocity = shooter.getDesiredFlywheelLinearVelocity();
        LinearVelocity currentLinearVelocity = shooter.getLinearVelocity();
        return currentLinearVelocity.minus(desiredLinearVelocity).in(MetersPerSecond)
                        <= -drop.in(MetersPerSecond)
                && currentLinearVelocity.in(MetersPerSecond)
                        > FlywheelConstants.TOLERANCE.in(RadiansPerSecond)
                                * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters)
                && shooter.staticShotState.getAsBoolean();
    }

    public static ShotTracker create(ShooterSuperstructure shooter) {
        return new ShotTracker(shooter);
    }
}
