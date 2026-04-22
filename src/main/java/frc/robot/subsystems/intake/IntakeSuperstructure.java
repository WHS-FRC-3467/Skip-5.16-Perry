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
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.lib.util.PowerProfiler;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {

    private final LinearMechanism<?> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;
    private final LoggedTrigger isRollerSafe;

    private final Debouncer rollerSafeDebouncer =
            new Debouncer(0.02, Debouncer.DebounceType.kRising);

    private static final LoggedTunableNumber MIN_SAFE_ROLLER_DISTANCE =
            new LoggedTunableNumber(
                    IntakeLinearConstants.NAME + "/MinSafeRollerDistanceInches", 5.0);

    public IntakeSuperstructure(
            LinearMechanism<?> intakeLinearIO, FlywheelMechanism<?> intakeRollerIO) {

        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        // Sets initial target position to fully retracted
        intakeLinearIO.runLinearPosition(
                IntakeLinearConstants.MIN_DISTANCE,
                PIDSlot.SLOT_0,
                IntakeLinearConstants.CRUISE_VELOCITY,
                IntakeLinearConstants.MAX_ACCELERATION);

        isExtended =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsExtended",
                        () ->
                                intakeLinearIO
                                        .getLinearPosition()
                                        .isNear(
                                                IntakeLinearConstants.MAX_DISTANCE,
                                                IntakeLinearConstants.TOLERANCE));

        isRetracted =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsRetracted",
                        () ->
                                intakeLinearIO
                                        .getLinearPosition()
                                        .isNear(
                                                IntakeLinearConstants.MIN_DISTANCE,
                                                IntakeLinearConstants.TOLERANCE));

        isRollerSafe =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsRollerSafe",
                        () ->
                                rollerSafeDebouncer.calculate(
                                        intakeLinearIO
                                                .getLinearPosition()
                                                .gte(Inches.of(MIN_SAFE_ROLLER_DISTANCE.get()))));
    }

    /** Returns true if the intake roller is running and the intake is extended. */
    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0
                && isExtended.getAsBoolean();
    }

    private Distance clampDistance(Distance goal) {
        return Meters.of(
                MathUtil.clamp(
                        goal.in(Meters),
                        IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                        IntakeLinearConstants.MAX_DISTANCE.in(Meters)));
    }

    /** Register the Intake subsystem with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName() + "/IntakeLinear", intakeLinearIO);
        powerProfiler.registerMechanism(getName() + "/IntakeRoller", intakeRollerIO);
    }

    /**
     * Moves the intake to a goal distance using Motion Magic with the given cruise velocity and
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
                                intakeLinearIO.runLinearPosition(
                                        clampedGoal, PIDSlot.SLOT_0, cruiseVelocity, acceleration))
                .withName(name);
    }

    private Command runRoller(double amps) {
        return this.runOnce(() -> intakeRollerIO.runCurrent(Amps.of(amps))).withName("Run Roller");
    }

    public Command stopRoller() {
        return this.runOnce(intakeRollerIO::runBrake).withName("Stop Roller");
    }

    public Command ejectRoller() {
        return this.startEnd(
                        () -> intakeRollerIO.runCurrent(Amps.of(-40.0)), intakeRollerIO::runBrake)
                .withName("Eject Roller");
    }

    public Command intake() {
        return Commands.sequence(
                        moveToPosition(
                                IntakeLinearConstants.MAX_DISTANCE,
                                IntakeLinearConstants.CRUISE_VELOCITY,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Extend Linear"),
                        Commands.waitUntil(isRollerSafe),
                        runRoller(80.0),
                        Commands.waitUntil(isExtended))
                .withName("Extend Intake With Roller");
    }

    public Command retractIntake() {
        return Commands.sequence(
                        moveToPosition(
                                IntakeLinearConstants.MIN_DISTANCE,
                                IntakeLinearConstants.CRUISE_VELOCITY,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Retract Linear"),
                        runRoller(80.0).onlyIf(isRollerSafe),
                        Commands.waitUntil(isRollerSafe.negate()),
                        stopRoller(),
                        Commands.waitUntil(isRetracted))
                .finallyDo(() -> intakeRollerIO.runBrake())
                .withName("Retract Intake With Roller");
    }

    public Command linearCoast() {
        return this.runOnce(intakeLinearIO::runCoast).withName("Linear Coast");
    }

    public Command homeLinear() {
        return Commands.sequence(
                        this.runOnce(() -> intakeLinearIO.runDutyCycle(0.25, true)), this.idle())
                .finallyDo(() -> intakeLinearIO.setEncoderPosition(Rotations.of(3.7)))
                .withName("Home Linear");
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        intakeLinearIO.periodic();
        intakeRollerIO.periodic();
    }

    /**
     * Gets the linear extension of the subsystem by converting the motor's rotation.
     *
     * @return The estimated linear extension of the subsystem
     */
    public Distance getExtension() {
        return intakeLinearIO.getLinearPosition();
    }

    @Override
    public void close() {
        intakeRollerIO.close();
        intakeLinearIO.close();
    }
}
