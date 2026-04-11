// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import frc.lib.util.AlwaysTunableNumber;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;

import java.util.Set;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger autonomous routines. Command logic layer.
 */
public class AutoCommands {
    private static final RobotState robotState = RobotState.getInstance();

    // Delay before following paths in auto.
    private static AlwaysTunableNumber autoDelay = new AlwaysTunableNumber("Auto/Delay", 0.0);

    /**
     * Accesses the value in the autoDelay AlwaysTunableNumber
     *
     * @return the delay, in seconds, to wait at the start of auto
     */
    public static double getAutoDelay() {
        return autoDelay.get();
    }

    public static Command shootCommand(
            Drive drive,
            IntakeSuperstructure intake,
            Indexer indexer,
            Tower tower,
            ShooterSuperstructure shooter,
            double timeoutDuration) {
        return Commands.deadline(
                Commands.parallel(
                                shooter.setShooterContinuous().asProxy(),
                                Commands.sequence(
                                        Commands.waitUntil(
                                                shooter.profileComplete.and(
                                                        RobotState.getInstance().facingTarget)),
                                        Commands.parallel(
                                                indexer.shoot(),
                                                tower.shoot(),
                                                Commands.waitSeconds(0.5)
                                                        .andThen(
                                                                Commands.defer(
                                                                        intake::slowRetract,
                                                                        Set.of(intake))))))
                        .until(robotState.hopperEmpty)
                        .withTimeout(timeoutDuration)
                        .finallyDo(
                                () -> {
                                    CommandScheduler.getInstance()
                                            .schedule(
                                                    shooter.setFlywheelSpeed(
                                                            RotationsPerSecond.zero()));
                                    CommandScheduler.getInstance()
                                            .schedule(shooter.setHoodAngle(Rotations.zero()));
                                }),
                DriveCommands.staticAimTowardsTarget(drive));
    }

    /**
     * Drive to the outpost (via pathCommand), and wait up to 3 seconds for FUEL to be dumped.
     *
     * @param pathCommand The command that follows the desired path to the outpost.
     * @return A command that follows a path (to the outpost), stops the robot, and waits 3 seconds.
     */
    public static Command driveAndCollectAtOutpost(Command pathCommand) {
        return Commands.sequence(
                pathCommand,
                Commands.waitSeconds(3),
                Commands.either(
                        Commands.runOnce(
                                () -> RobotSim.getInstance().getFuelSim().fillHopperBy(20)),
                        Commands.none(),
                        RobotBase::isSimulation));
    }

    /**
     * Makes the robot smaller by retracting the intake and lowering the hood.
     *
     * @param shooter the shooter superstructure subsystem
     * @return returns a timed command that retracts the intake and lowers the hood
     */
    public static Command stowHood(ShooterSuperstructure shooter) {
        return Commands.parallel(shooter.retractHood()).withTimeout(1.25);
    }

    /**
     * Shoots the currently held FUEL, stows the hood in the background, then starts the supplied
     * trajectory-following command.
     *
     * <p>StowHood is wrapped in a {@link ScheduleCommand} so its {@code ShooterSuperstructure}
     * requirement is not added to this sequence, keeping subsystem requirements minimal for the
     * trajectory-following phase.
     *
     * @param ctx the auto context providing subsystem references
     * @param timeoutSeconds maximum time to wait for the shot
     * @param nextTrajectory the command to follow after shooting (typically a resilient trajectory
     *     follower)
     * @return a command that shoots, stows the hood independently, then follows the next trajectory
     */
    public static Command shootThenFollow(
            AutoContext ctx, double timeoutSeconds, Command nextTrajectory) {
        return Commands.sequence(
                shootOnly(ctx, timeoutSeconds),
                new ScheduleCommand(stowHood(ctx.shooter())),
                nextTrajectory);
    }

    private static Command shootOnly(AutoContext ctx, double timeoutSeconds) {
        return shootCommand(
                ctx.drive(),
                ctx.intake(),
                ctx.indexer(),
                ctx.tower(),
                ctx.shooter(),
                timeoutSeconds);
    }

    private static Command retractIntake(AutoContext ctx) {
        return ctx.intake().retractIntake().withTimeout(0.5);
    }
}
