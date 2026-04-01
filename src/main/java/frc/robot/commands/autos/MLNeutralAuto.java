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

/** Native Choreo routine for the neutral-zone multi-piece autonomous variants. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class MLNeutralAuto {
    private static final Alert TRAJECTORIES_MISSING =
            new Alert(
                    "ML Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);
    private static final Alert OBJECT_DETECTOR_MISSING =
            new Alert(
                    "ML Neutral Auto Object Detector Missing, Auto Unavailable", AlertType.kError);
    private static AutoTrajectory laneTrajectory = null;

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

        Optional<Trajectory<SwerveSample>> tunnelTrajectory =
                AutoUtil.loadTrajectory(ChoreoTraj.TunnelPath.name(), shouldMirror);

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
                            Optional<AutoTrajectory> tunnel =
                                    tunnelTrajectory.map(routine::trajectory);

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

                            start.done()
                                    .onTrue(
                                            AutoCommands.shootThenFollow(
                                                    ctx, 3.0, decision)); // works up to here

                            // ML loop: decicde on lane -> follow best lane -> shoot -> path to
                            // decision pose
                            decision.done()
                                    .onTrue(
                                            Commands.sequence(
                                                            Commands.waitSeconds(0.1),
                                                            Commands.runOnce(
                                                                    () -> {
                                                                        int lane =
                                                                                AutoCommands
                                                                                        .getBestLane(
                                                                                                objectDetector)
                                                                                        .orElse(-1);
                                                                        laneTrajectory =
                                                                                switch (lane) {
                                                                                    case 0 ->
                                                                                            laneOne;
                                                                                    case 1 ->
                                                                                            laneTwo;
                                                                                    case 2 ->
                                                                                            laneThree;
                                                                                    default ->
                                                                                            fallback;
                                                                                };
                                                                        Logger.recordOutput(
                                                                                "Detection/ChosenLane",
                                                                                laneTrajectory
                                                                                        .toString());
                                                                    }),
                                                            Commands.defer(
                                                                    () ->
                                                                            AutoCommands
                                                                                    .followThenShoot(
                                                                                            ctx,
                                                                                            3.0,
                                                                                            laneTrajectory),
                                                                    Set.of()),
                                                            decision.spawnCmd(),
                                                            Commands.waitUntil(decision.done()))
                                                    .repeatedly());

                            return routine;
                        }));
    }
}
