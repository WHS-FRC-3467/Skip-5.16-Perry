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
package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;
import frc.robot.subsystems.objectdetector.ObjectDetector;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/** Native Choreo routine for the neutral-zone multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class MLNeutralAuto {
    private static final Alert TRAJECTORIES_MISSING =
            new Alert(
                    "ML Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);
    private static final Alert OBJECT_DETECTOR_MISSING =
            new Alert(
                    "ML Neutral Auto Object Detector Missing, Auto Unavailable", AlertType.kError);

    /**
     * Builds the selected neutral ML auto variant.
     *
     * @param shouldMirror Whether to mirror the route for the opposite starting side
     * @param isSafe Whether to use the safer first segment instead of the aggressive one
     */
    public static Optional<AutoOption> create(
            AutoContext ctx, boolean shouldMirror, boolean isSafe) {
        List<String> names =
                isSafe
                        ? List.of(
                                ChoreoTraj.NeutralBump_ML_Start.name(),
                                ChoreoTraj.NeutralBump_ML_SafeRight.name(),
                                ChoreoTraj.NeutralBump_ML_Decision.name(),
                                ChoreoTraj.NeutralBump_ML_SafeLeft.name(),
                                ChoreoTraj.NeutralBump_ML_SafeMiddle.name(),
                                ChoreoTraj.NeutralBump_ML_SafeRight.name())
                        : List.of(
                                ChoreoTraj.NeutralBump_ML_Start.name(), // Placeholders for now
                                ChoreoTraj.NeutralBump_ML_SafeRight.name(),
                                ChoreoTraj.NeutralBump_ML_Decision.name(),
                                ChoreoTraj.NeutralBump_ML_SafeLeft.name(),
                                ChoreoTraj.NeutralBump_ML_SafeMiddle.name(),
                                ChoreoTraj.NeutralBump_ML_SafeRight.name());
        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, shouldMirror).orElse(null);
        if (trajectories == null) {
            TRAJECTORIES_MISSING.set(true);
            return Optional.empty();
        }
        if (ctx.objectDetector().isEmpty()) {
            OBJECT_DETECTOR_MISSING.set(true);
            return Optional.empty();
        }
        ObjectDetector objectDetector = ctx.objectDetector().get();

        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine(
                                                    "MLNeutral"
                                                            + (isSafe ? "Safe" : "Aggressive")
                                                            + (shouldMirror ? "Right" : "Left"));

                            AutoTrajectory start = routine.trajectory(trajectories.get(0));
                            AutoTrajectory fallback = routine.trajectory(trajectories.get(1));
                            AutoTrajectory decision = routine.trajectory(trajectories.get(2));
                            AutoTrajectory laneOne = routine.trajectory(trajectories.get(3));
                            AutoTrajectory laneTwo = routine.trajectory(trajectories.get(4));
                            AutoTrajectory laneThree = routine.trajectory(trajectories.get(5));

                            // Atomic types act as atomically mutable owned references to heap
                            // memory on the stack,
                            // similar to Box in Rust or std::unique_ptr in C++. Lambdas capture the
                            // reference
                            // rather than the value, allowing for interactions similar to class
                            // members in a lambda.

                            // Notifier for when to make a descision. There should only be one
                            // reader,
                            // and it should always set this back to false immediately after it has
                            // finished.
                            AtomicBoolean queueDecision = new AtomicBoolean(false);

                            // The descision that has been made
                            AtomicReference<AutoTrajectory> selectedLane =
                                    new AtomicReference<>(fallback);

                            AutoUtil.bindEvents(
                                    ctx, start, decision, laneOne, laneTwo, laneThree, fallback);

                            routine.active()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.runOnce(
                                                            ctx.drive()
                                                                    ::resetTrajectoryControllers),
                                                    start.resetOdometry(),
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getAutoDelay()),
                                                            Set.of()),
                                                    start.spawnCmd()));

                            routine.observe(queueDecision::get)
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.runOnce(
                                                            () -> queueDecision.set(false)),
                                                    decision.spawnCmd()));

                            start.done()
                                    .onTrue(
                                            Commands.sequence(
                                                    AutoCommands.shootThenPrep(ctx, 3.0),
                                                    Commands.runOnce(
                                                            () -> queueDecision.set(true))));

                            decision.done()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.waitSeconds(0.05),
                                                    Commands.runOnce(
                                                            () -> {
                                                                int lane =
                                                                        AutoCommands.getBestLane(
                                                                                        objectDetector)
                                                                                .orElse(-1);
                                                                selectedLane.set(
                                                                        switch (lane) {
                                                                            case 0 -> laneOne;
                                                                            case 1 -> laneTwo;
                                                                            case 2 -> laneThree;
                                                                            default -> fallback;
                                                                        });
                                                                Logger.recordOutput(
                                                                        "Detection/ChosenLane",
                                                                        lane);
                                                            }),
                                                    Commands.defer(
                                                            () ->
                                                                    AutoCommands.followThenPrep(
                                                                            ctx,
                                                                            3.0,
                                                                            selectedLane.get()),
                                                            Set.of()),
                                                    Commands.runOnce(
                                                            () -> queueDecision.set(true))));

                            return routine;
                        }));
    }
}
