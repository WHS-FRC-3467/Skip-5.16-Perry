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

import frc.robot.commands.ResilientTrajectoryFollower;
import frc.robot.commands.autos.utils.AutoCommands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;
import frc.robot.generated.ChoreoTraj;

import java.util.List;
import java.util.Map;
import java.util.Optional;

public class DepotAuto {

    private static final Alert TRAJECTORIES_MISSING =
            new Alert("Depot Auto Trajectories Missing, Auto(s) Unavailable", AlertType.kError);

    public static Optional<AutoOption> create(
            AutoContext ctx, boolean shouldMirror, boolean isSafe) {
        List<String> names = List.of(ChoreoTraj.Depot1.name());

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
                            AutoRoutine routine = ctx.autoFactory().newRoutine("Depot Auto");

                            // Still use AutoTrajectory for resetOdometry() lifecycle.
                            AutoTrajectory first = routine.trajectory(trajectories.get(0));

                            Map<String, Command> eventBindings = AutoUtil.createEventBindings(ctx);

                            // Declare the trajectory-following command up front so we
                            // can grab the .done() trigger.
                            ResilientTrajectoryFollower firstFollow =
                                    ctx.drive()
                                            .followTrajectoryResilient(
                                                    trajectories.get(0), eventBindings);

                            // Phase 1: Reset controllers, odometry, wait for delay,
                            // then follow the first trajectory. Because the sequence
                            // only contains Drive-requiring commands, the event-bound
                            // commands (Intake, Shooter) can schedule without conflict.
                            routine.active()
                                    .onTrue(Commands.sequence(first.resetOdometry(), firstFollow));

                            // Phase 2: When the trajectory completes, shoot.
                            firstFollow.done().onTrue(AutoCommands.shootOnly(ctx, 5.0));

                            return routine;
                        }));
    }
}
