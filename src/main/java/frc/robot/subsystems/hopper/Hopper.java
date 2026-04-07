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
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

import java.util.function.Supplier;

public class Hopper extends SubsystemBase implements AutoCloseable {

    private static final LoggedTunableNumber SLOW_MPS =
            new LoggedTunableNumber(HopperConstants.NAME + "/SlowMPS", 0.25);

    private final LinearMechanism<?> io;

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;

    private final LinearVelocity shuffleVelocity = MetersPerSecond.of(0.8);
    private final Distance retractDistance = HopperConstants.MIN_DISTANCE;

    public Hopper(LinearMechanism<?> io) {

        this.io = io;
        io.runLinearPosition(
                HopperConstants.MIN_DISTANCE,
                PIDSlot.SLOT_0,
                HopperConstants.CRUISE_VELOCITY,
                HopperConstants.MAX_ACCELERATION);

        isExtended =
                new LoggedTrigger(
                        "Hopper/IsExtended",
                        () ->
                                MathUtil.isNear(
                                        HopperConstants.MAX_DISTANCE.in(Meters),
                                        io.getLinearPosition().in(Meters),
                                        HopperConstants.TOLERANCE.in(Meters)));

        isRetracted =
                new LoggedTrigger(
                        "Hopper/IsRetracted",
                        () ->
                                MathUtil.isNear(
                                        retractDistance.in(Meters),
                                        io.getLinearPosition().in(Meters),
                                        HopperConstants.TOLERANCE.in(Meters)));
    }

    /** Returns true if the hopper is not retracted . */
    public boolean isNotRetracted() {
        return !isRetracted.getAsBoolean();
    }

    /** Returns true if the hopper is vertically extended . */
    public boolean isExtended() {
        return isExtended.getAsBoolean();
    }

    /**
     * Moves the hopper to a goal distance using Motion Magic with the given cruise velocity and
     * acceleration. The command completes immediately after issuing the control request. Use {@link
     * Commands#waitUntil} to block until the goal is reached.
     */
    private Command moveToPosition(
            Distance goal,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            String name) {
        Distance clampedGoal = clampDistance(goal);
        return this.runOnce(
                        () ->
                                io.runLinearPosition(
                                        clampedGoal, PIDSlot.SLOT_0, cruiseVelocity, acceleration))
                .withName(name);
    }

    private Distance clampDistance(Distance goal) {
        return Meters.of(
                MathUtil.clamp(
                        goal.in(Meters),
                        HopperConstants.MIN_DISTANCE.in(Meters),
                        HopperConstants.MAX_DISTANCE.in(Meters)));
    }

    /**
     * Moves the hopper by a delta using Motion Magic with the given cruise velocity and
     * acceleration, optionally running the roller at a scaled speed, and waits until within
     * toleranceMeters of the goal.
     */
    private Command moveByDistance(
            Distance distance,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            double toleranceMeters,
            String name) {
        return moveToDistance(
                () -> Meters.of(io.getLinearPosition().in(Meters) + distance.in(Meters)),
                cruiseVelocity,
                acceleration,
                toleranceMeters,
                name);
    }

    private Command moveToDistance(
            Supplier<Distance> goalSupplier,
            LinearVelocity cruiseVelocity,
            LinearAcceleration acceleration,
            double toleranceMeters,
            String name) {
        return Commands.sequence(
                        this.runOnce(
                                () ->
                                        io.runLinearPosition(
                                                clampDistance(goalSupplier.get()),
                                                PIDSlot.SLOT_0,
                                                cruiseVelocity,
                                                acceleration)),
                        Commands.waitUntil(
                                () ->
                                        MathUtil.isNear(
                                                io.getGoalLinearPosition().in(Meters),
                                                io.getLinearPosition().in(Meters),
                                                toleranceMeters)))
                .withName(name);
    }

    /**
     * Extend the hopper.
     *
     * @return a command to extend the hopper as fast as possible
     */
    public Command extend() {
        return Commands.sequence(
                        moveToPosition(
                                HopperConstants.MAX_DISTANCE,
                                HopperConstants.CRUISE_VELOCITY,
                                HopperConstants.MAX_ACCELERATION,
                                "Extend Hopper"))
                .withName("Extend Hopper");
    }

    /**
     * Retract the hopper.
     *
     * @return a command to retract the hopper as fast as possible
     */
    public Command fastRetract() {
        return Commands.sequence(
                        moveToPosition(
                                HopperConstants.MIN_DISTANCE,
                                HopperConstants.CRUISE_VELOCITY,
                                HopperConstants.MAX_ACCELERATION,
                                "Fast Retract Hopper"))
                .withName("Fast Retract Hopper");
    }

    public Command cruiseRetract() {
        return retractWithSpeed(HopperConstants.CRUISE_VELOCITY, "Retract Hopper");
    }

    public Command slowRetract(LinearVelocity retractSpeed) {
        return retractWithSpeed(retractSpeed, "Slow Retract");
    }

    public Command slowRetract() {
        return slowRetract(MetersPerSecond.of(SLOW_MPS.get()));
    }

    private Command retractWithSpeed(LinearVelocity retractSpeed, String name) {
        return Commands.sequence(
                        moveToPosition(
                                retractDistance,
                                retractSpeed,
                                HopperConstants.MAX_ACCELERATION,
                                "Retract Linear"),
                        Commands.waitUntil(isRetracted))
                .withName(name);
    }

    public Command linearCoast() {
        return this.runOnce(io::runCoast).withName("Linear Coast");
    }

    public Command homeLinear() {
        return Commands.sequence(this.runOnce(() -> io.runDutyCycle(0.25, true)), this.idle())
                .finallyDo(() -> io.setEncoderPosition(Rotations.of(3.7)))
                .withName("Home Hopper");
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        io.periodic();
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return io.getLinearPosition();
    }

    @Override
    public void close() {
        io.close();
    }
}
