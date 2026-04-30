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
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.ResilientTrajectoryFollower;
import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class FullNeutralAuto {

    private static final Alert TRAJECTORIES_MISSING =
            new Alert(
                    "Full Neutral Auto Trajectories Missing, Auto(s) Unavailable",
                    AlertType.kError);

    public static Optional<AutoOption> create(AutoContext ctx) {
        List<String> names =
                List.of(
                        ChoreoTraj.FullNeutral1.name(),
                        ChoreoTraj.FullNeutral2.name(),
                        ChoreoTraj.FullNeutral3.name());

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
                            AutoRoutine routine = ctx.autoFactory().newRoutine("FullNeutralAuto");

                            Map<String, Command> eventBindings = AutoUtil.createEventBindings(ctx);

                            ResilientTrajectoryFollower firstFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(0), eventBindings);
                            ResilientTrajectoryFollower secondFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(1), eventBindings);
                            ResilientTrajectoryFollower thirdFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(2), eventBindings);

                            routine.active()
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getStartDelay()),
                                                            Set.of()),
                                                    firstFollow));

                            routine.observe(firstFollow.done())
                                    .onTrue(
                                            Commands.sequence(
                                                    Commands.defer(
                                                            () ->
                                                                    Commands.waitSeconds(
                                                                            AutoCommands
                                                                                    .getBumpDelay()),
                                                            Set.of()),
                                                    secondFollow.asProxy()));

                            routine.observe(secondFollow.done())
                                    .onTrue(
                                            Commands.sequence(
                                                    AutoCommands.shootOnly(ctx, 5.0),
                                                    thirdFollow.asProxy()));

                            routine.observe(thirdFollow.done())
                                    .onTrue(Commands.sequence(AutoCommands.shootOnly(ctx, 5.0)));

                            return routine;
                        }));
    }
}
