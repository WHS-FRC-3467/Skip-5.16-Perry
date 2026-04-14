// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.AlwaysTunableNumber;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;

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
                                                                        intake::retractIntake,
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

    // /**
    //  * Return the int corresponding to the lane most populated with FUEL according to the object
    //  * detector, if the lane exists. Currently factored for just 3 ML lanes indexed 0-2.
    //  *
    //  * @param objectDetector the object detector subsystem
    //  * @return an Optional containing the lane number (0, 1, or 2) with the most FUEL, or an
    // empty
    //  *     Optional if no lane is matched
    //  */
    // public static Optional<Integer> getBestLane(ObjectDetector objectDetector) {
    //     Optional<LaneTarget> bestLaneTarget = objectDetector.getBestLaneTarget();
    //     if (bestLaneTarget.isEmpty()) {
    //         return Optional.empty();
    //     }
    //     double laneX = bestLaneTarget.get().x();

    //     // Three pre-defined ML lanes, so find the closest one to the optimal lane
    //     double[] lanes =
    //             new double[] {
    //                 ChoreoVars.Poses.LanePose1ML.getX(),
    //                 ChoreoVars.Poses.LanePose2ML.getX(),
    //                 ChoreoVars.Poses.LanePose3ML.getX()
    //             };

    //     int bestIndex = -1;
    //     double bestDistance = Double.MAX_VALUE;
    //     for (int i = 0; i < lanes.length; i++) {
    //         double distance = Math.abs(laneX - lanes[i]);
    //         if (distance < bestDistance) {
    //             bestDistance = distance;
    //             bestIndex = i;
    //         }
    //     }

    //     return bestIndex == -1 ? Optional.empty() : Optional.of(bestIndex);
    // }

    public static Command shootOnly(AutoContext ctx, double timeoutSeconds) {
        return shootCommand(
                ctx.drive(),
                ctx.intake(),
                ctx.indexer(),
                ctx.tower(),
                ctx.shooter(),
                timeoutSeconds);
    }
}
