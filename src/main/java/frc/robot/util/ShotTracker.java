// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.LoggedInt;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.FlywheelConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;

import lombok.Getter;

public class ShotTracker {

    private final ShooterSuperstructure shooter;
    private final RobotState robotState = RobotState.getInstance();

    // Fuel counts
    private @Getter int totalFuelCount = 0;

    // Linear velocity drop required to detect a shot passing through the shooter, default tuned
    // from auto replay logs. Typically 0.5 - 1 m/s.
    private final LoggedTunableNumber shotDetectionThresholdMPS =
            new LoggedTunableNumber("ShotTracker/ShotDetectionThresholdMPS", 0.30);

    // Triggers determining whether a ball has passed through the shooter based on flywheel velocity
    // drops from current setpoint, currently only registering true during static feeding/shooting
    private final LoggedTrigger ballTrigger =
            new LoggedTrigger(
                    "ShotTracker/BallTrigger",
                    () ->
                            detectFlywheelDrop(
                                    MetersPerSecond.of(shotDetectionThresholdMPS.getAsDouble())));

    // Determines whether the hopper is empty for at least 0.575s while shooting, using
    // staticShotState as a proxy for a shot
    private final Debouncer hopperEmptyDebouncer = new Debouncer(0.575, DebounceType.kRising);
    private final LoggedTrigger hopperEmpty;
    // Logged fuel counter to avoid repeated identical writes
    private final LoggedInt totalFuelLogger;

    public ShotTracker(ShooterSuperstructure shooter) {
        this.shooter = shooter;
        this.totalFuelLogger = new LoggedInt("ShotTracker/TotalFuelCount");

        hopperEmpty =
                RobotBase.isSimulation()
                        ? new LoggedTrigger(
                                "ShotTracker/hopperEmpty",
                                () ->
                                        hopperEmptyDebouncer.calculate(
                                                RobotSim.getInstance().getFuelSim().getHeldFuel()
                                                        == 0))
                        : new LoggedTrigger(
                                "ShotTracker/hopperEmpty",
                                () ->
                                        hopperEmptyDebouncer.calculate(
                                                this.shooter.staticShotState.getAsBoolean()
                                                        && !ballTrigger.getAsBoolean()));
        robotState.setHopperEmpty(hopperEmpty);
        attachBallTriggers();
    }

    private void attachBallTriggers() {
        ballTrigger.onTrue(
                Commands.runOnce(
                        () -> {
                            totalFuelCount++;
                            // Log count only on change
                            totalFuelLogger.log(totalFuelCount);
                        }));
    }

    /**
     * Determines whether left flywheel linear velocity has dropped by at least the specified
     * velocity from the current flywheel linear velocity setpoint. Currently only applicable during
     * static feeding/shooting. Primarily for use in autos.
     *
     * <p>Gating the check behind having the flywheel be above a certain minimum velocity and the
     * static shot state helps prevent false positives from spurious velocity drops when the
     * flywheel is at low speed or the robot is moving/spinning up.
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

    public static void create(ShooterSuperstructure shooter) {
        new ShotTracker(shooter);
    }
}
