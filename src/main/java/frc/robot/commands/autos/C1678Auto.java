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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class C1678Auto {

    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Neutral Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    public static Optional<AutoOption> create(
            AutoContext ctx, boolean shouldMirror, boolean isSafe) {
        List<String> names =
                isSafe
                        ? List.of(ChoreoTraj.C1678Safe1.name(), ChoreoTraj.C16782.name())
                        : List.of(ChoreoTraj.C16781.name(), ChoreoTraj.C16782.name());

        List<Trajectory<SwerveSample>> trajectories =
                AutoUtil.loadTrajectories(names, shouldMirror).orElse(null);

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
                                                    "STSE"
                                                            + (isSafe ? "Safe" : "Aggressive")
                                                            + (shouldMirror ? "Right" : "Left"));

                            // Still use AutoTrajectory for resetOdometry() lifecycle.
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));

                            Map<String, Command> eventBindings = AutoUtil.createEventBindings(ctx);

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
                                                    ctx.drive()
                                                            .followTrajectoryResilient(
                                                                    trajectories.get(0),
                                                                    eventBindings),
                                                    AutoCommands.shootCommand(
                                                            ctx.drive(),
                                                            ctx.intake(),
                                                            ctx.indexer(),
                                                            ctx.tower(),
                                                            ctx.shooter(),
                                                            3.0),
                                                    AutoCommands.stowHood(ctx.shooter()),
                                                    ctx.drive()
                                                            .followTrajectoryResilient(
                                                                    trajectories.get(1),
                                                                    eventBindings),
                                                    AutoCommands.shootCommand(
                                                            ctx.drive(),
                                                            ctx.intake(),
                                                            ctx.indexer(),
                                                            ctx.tower(),
                                                            ctx.shooter(),
                                                            10.0)));

                            return routine;
                        }));
    }
}
