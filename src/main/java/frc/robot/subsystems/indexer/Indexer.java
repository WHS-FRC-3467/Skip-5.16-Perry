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
package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;

/**
 * Subsystem that controls the indexer floor and indexer centering mechanism for moving game pieces
 * within the robot. The indexer can pull game pieces in, expel them, or stop. Uses a flywheel
 * mechanism for velocity control.
 */
public class Indexer extends SubsystemBase {
    private final FlywheelMechanism<?> io;

    private static final LoggedTunableNumber SHOOT_RPS =
            new LoggedTunableNumber(
                    IndexerConstants.NAME + "/ShootRPS",
                    IndexerConstants.MAX_VELOCITY.in(RotationsPerSecond));

    private static final LoggedTunableNumber EJECT_RPS =
            new LoggedTunableNumber(IndexerConstants.NAME + "/EjectRPS", -30.0);

    /**
     * Constructs an Indexer subsystem.
     *
     * @param io The flywheel mechanism for controlling the indexer floor motors
     */
    public Indexer(FlywheelMechanism<?> io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggerHelper.recordCurrentCommand(this.getName(), this);
        io.periodic();
    }

    private void stop() {
        io.runCoast();
    }

    private void runVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity, PIDSlot.SLOT_0);
    }

    /**
     * Creates a command to stop the indexer by applying coast mode.
     *
     * @return a command that stops the indexer
     */
    public Command stopCommand() {
        return this.runOnce(this::stop).withName("Stop Indexer");
    }

    /**
     * Run the indexer at the fountain velocity
     *
     * @return a command to fountain
     */
    public Command fountain() {
        return this.runOnce(() -> runVelocity(RotationsPerSecond.of(5.0)));
    }

    /**
     * Creates a command to run the indexer at shooting velocities. The indexer will stop when the
     * command is interrupted or cancelled.
     *
     * @return a command that runs the indexer at shooting speed
     */
    public Command shoot() {
        return this.startEnd(
                        () -> runVelocity(RotationsPerSecond.of(SHOOT_RPS.get())), () -> stop())
                .withName("Shoot");
    }

    /**
     * Creates a command to run the indexer in reverse to eject game pieces. The indexer will stop
     * when the command is interrupted or cancelled.
     *
     * @return a command that runs the indexer in reverse
     */
    public Command eject() {
        return this.startEnd(
                        () -> runVelocity(RotationsPerSecond.of(EJECT_RPS.get())), () -> stop())
                .withName("Eject");
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    /**
     * Checks if the indexer velocity is near the current state's setpoint.
     *
     * @return true if the indexer is within tolerance of the setpoint, false otherwise
     */
    public boolean nearSetpoint() {
        return io.getVelocityError().lte(IndexerConstants.TOLERANCE);
    }

    /** Closes the indexer mechanism and releases resources. */
    public void close() {
        io.close();
    }
}
