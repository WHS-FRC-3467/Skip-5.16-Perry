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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

public class IntakeSuperstructure extends SubsystemBase implements AutoCloseable {

    private final LinearMechanism<?> intakeLinearIO;
    private final FlywheelMechanism<?> intakeRollerIO;

    private static final LoggedTunableNumber ROLLER_EJECT_RPS =
            new LoggedTunableNumber(IntakeRollerConstants.NAME + "/EjectRPS", -35.0);

    private static final LoggedTunableNumber SLOW_MPS =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/SlowMPS", 0.25);

    /**
     * Minimum safe roller distance from the retracted position such that the roller doesn't
     * interfere with surrounding hardware.
     */
    private static final LoggedTunableNumber MIN_SAFE_ROLLER_DISTANCE =
            new LoggedTunableNumber(IntakeLinearConstants.NAME + "/MinSafeRollerDistance", 0.05);

    private final LoggedTrigger isExtended;
    private final LoggedTrigger isRetracted;
    private final LoggedTrigger isRollerSafe;

    private final Distance retractDistance = IntakeLinearConstants.MIN_DISTANCE;

    public IntakeSuperstructure(
            LinearMechanism<?> intakeLinearIO, FlywheelMechanism<?> intakeRollerIO) {

        this.intakeLinearIO = intakeLinearIO;
        this.intakeRollerIO = intakeRollerIO;

        intakeLinearIO.runLinearPosition(
                IntakeLinearConstants.MIN_DISTANCE,
                PIDSlot.SLOT_0,
                IntakeLinearConstants.CRUISE_VELOCITY,
                IntakeLinearConstants.MAX_ACCELERATION);

        isExtended =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsExtended",
                        () ->
                                MathUtil.isNear(
                                        IntakeLinearConstants.MAX_DISTANCE.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));

        isRetracted =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsRetracted",
                        () ->
                                MathUtil.isNear(
                                        retractDistance.in(Meters),
                                        intakeLinearIO.getLinearPosition().in(Meters),
                                        IntakeLinearConstants.TOLERANCE.in(Meters)));

        isRollerSafe =
                new LoggedTrigger(
                        "IntakeSuperstructure/IsRollerSafe",
                        () ->
                                intakeLinearIO.getLinearPosition().in(Meters)
                                        > MIN_SAFE_ROLLER_DISTANCE.get());
    }

    /** Returns true if the intake roller is running and the intake is extended. */
    public boolean isIntaking() {
        return intakeRollerIO.getVelocity().in(RotationsPerSecond) > 1.0
                && isExtended.getAsBoolean();
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

    private Distance clampDistance(Distance goal) {
        return Meters.of(
                MathUtil.clamp(
                        goal.in(Meters),
                        IntakeLinearConstants.MIN_DISTANCE.in(Meters),
                        IntakeLinearConstants.MAX_DISTANCE.in(Meters)));
    }

    private Command runRoller(double amps) {
        return Commands.either(
                        Commands.runOnce(() -> intakeRollerIO.runCurrent(Amps.of(amps)), this),
                        stopRoller(),
                        isRollerSafe)
                .withName("Run Roller");
    }

    public Command stopRoller() {
        return this.runOnce(intakeRollerIO::runBrake).withName("Stop Roller");
    }

    public Command ejectRoller() {
        return this.startEnd(
                        () ->
                                intakeRollerIO.runVelocity(
                                        RotationsPerSecond.of(ROLLER_EJECT_RPS.get()),
                                        PIDSlot.SLOT_0),
                        intakeRollerIO::runBrake)
                .withName("Eject Roller");
    }

    public Command intake() {
        return Commands.sequence(
                        runRoller(80.0),
                        moveToPosition(
                                IntakeLinearConstants.MAX_DISTANCE,
                                IntakeLinearConstants.CRUISE_VELOCITY,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Extend Linear"))
                .withName("Extend With Roller");
    }

    public Command retractIntake() {
        return retractWithSpeed(IntakeLinearConstants.CRUISE_VELOCITY).withName("Retract Intake");
    }

    public Command slowRetract() {
        return retractWithSpeed(MetersPerSecond.of(SLOW_MPS.get())).withName("Slow Retract Intake");
    }

    private Command retractWithSpeed(LinearVelocity retractSpeed) {
        return Commands.sequence(
                        runRoller(80.0),
                        moveToPosition(
                                retractDistance,
                                retractSpeed,
                                IntakeLinearConstants.MAX_ACCELERATION,
                                "Retract Linear"),
                        Commands.waitUntil(isRetracted),
                        stopRoller())
                .finallyDo(() -> intakeRollerIO.runBrake())
                .withName("Retract With Speed");
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
