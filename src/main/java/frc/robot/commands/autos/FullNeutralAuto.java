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

import lombok.AllArgsConstructor;

import java.util.List;
import java.util.Optional;
import java.util.Set;

public class FullNeutralAuto {

    @AllArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum Positions {
        Bump(true),
        Trench(false);
        private final boolean position;

        public boolean get() {
            return position;
        }
    }

    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    public static Optional<AutoOption> create(
            AutoContext ctx, Positions startPosition, Positions returnPosition) {
        List<String> names =
                List.of(
                        startPosition.get()
                                ? ChoreoTraj.FullNeutralBump1.name()
                                : ChoreoTraj.FullNeutralTrench1.name(),
                        returnPosition.get()
                                ? ChoreoTraj.FullNeutralBump2.name()
                                : ChoreoTraj.FullNeutralTrench2.name(),
                        returnPosition.get()
                                ? ChoreoTraj.BumpPath.name()
                                : ChoreoTraj.TunnelPath.name(),
                        returnPosition.get()
                                ? ChoreoTraj.FullNeutralBump3.name()
                                : ChoreoTraj.FullNeutralTrench3.name());

        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, false).orElse(null);
        if (trajectories == null) {
            TRAJECTORIES_MISSING.set(true);
            return Optional.empty();
        }
        return Optional.of(
                AutoUtil.trajectoryOption(
                        trajectories,
                        () -> {
                            AutoRoutine routine =
                                    ctx.autoFactory()
                                            .newRoutine(
                                                    "FullNeutral"
                                                            + (startPosition.get()
                                                                    ? " Bump"
                                                                    : " Trench")
                                                            + (returnPosition.get()
                                                                    ? " Bump"
                                                                    : " Trench"));

                            AutoTrajectory first = routine.trajectory(trajectories.get(0));
                            AutoTrajectory second = routine.trajectory(trajectories.get(1));
                            AutoTrajectory third = routine.trajectory(trajectories.get(2));
                            AutoTrajectory fourth = routine.trajectory(trajectories.get(3));
                            AutoUtil.bindEvents(ctx, first, second, third, fourth);

                            routine.active()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.runOnce(
                                                            ctx.drive()
                                                                    ::resetTrajectoryControllers),
                                                    first.resetOdometry(),
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getAutoDelay()),
                                                            Set.of()),
                                                    first.spawnCmd()));

                            first.done().onTrue(second.spawnCmd());
                            AutoCommands.recoverThenFollow(
                                    ctx, first, Optional.of(second), 10.0, third);
                            second.done().onTrue(third.spawnCmd());
                            AutoCommands.recoverThenFollow(
                                    ctx, second, Optional.of(third), 10.0, fourth);
                            third.done().onTrue(AutoCommands.shootThenFollow(ctx, 10.0, fourth));
                            fourth.done()
                                    .onTrue(
                                            Commands.sequence(
                                                    AutoCommands.shootCommand(
                                                            ctx.drive(),
                                                            ctx.intake(),
                                                            ctx.indexer(),
                                                            ctx.tower(),
                                                            ctx.shooter(),
                                                            10.0),
                                                    ctx.shooter().retractHood()));

                            return routine;
                        }));
    }
}
