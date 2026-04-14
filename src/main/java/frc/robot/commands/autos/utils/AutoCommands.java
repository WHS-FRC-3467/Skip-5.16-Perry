// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.AlwaysTunableNumber;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldRegion;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.objectdetector.ObjectDetector;
import frc.robot.subsystems.objectdetector.ObjectDetector.LaneTarget;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

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
     * Shoots the currently held note, stows the hood, retracts the intake, then starts the next
     * trajectory.
     */
    public static Command shootThenFollow(
            AutoContext ctx, double timeoutSeconds, AutoTrajectory next) {
        return Commands.sequence(
                shootOnly(ctx, timeoutSeconds),
                new ScheduleCommand(stowHood(ctx.shooter())),
                next.spawnCmd());
    }

    /**
     * Follows the given trajectory then shoots the currently held FUEL. Spins down the shooter and
     * retracts the intake afterwards.
     */
    public static Command followThenShoot(
            AutoContext ctx, double timeoutSeconds, AutoTrajectory current) {
        return Commands.sequence(current.cmd(), shootOnly(ctx, timeoutSeconds), retractIntake(ctx));
    }

    /**
     * Shoots the held note, then performs the post-shot cleanup before another trigger continues.
     */
    public static Command shootThenPrep(AutoContext ctx, double timeoutSeconds) {
        return Commands.sequence(
                shootOnly(ctx, timeoutSeconds), stowHood(ctx.shooter()), retractIntake(ctx));
    }

    /**
     * Return the int corresponding to the lane most populated with FUEL according to the object
     * detector, if the lane exists. Currently factored for just 3 ML lanes indexed 0-2.
     *
     * @param objectDetector the object detector subsystem
     * @return an Optional containing the lane number (0, 1, or 2) with the most FUEL, or an empty
     *     Optional if no lane is matched
     */
    public static Optional<Integer> getBestLane(ObjectDetector objectDetector) {
        Optional<LaneTarget> bestLaneTarget = objectDetector.getBestLaneTarget();
        if (bestLaneTarget.isEmpty()) {
            return Optional.empty();
        }
        double laneX = bestLaneTarget.get().x();

        // Three pre-defined ML lanes, so find the closest one to the optimal lane
        boolean shouldFlip = FieldUtil.shouldFlip();
        double x1 = ChoreoVars.Poses.LanePose1ML.getMeasureX().in(Meters);
        double x2 = ChoreoVars.Poses.LanePose2ML.getMeasureX().in(Meters);
        double x3 = ChoreoVars.Poses.LanePose3ML.getMeasureX().in(Meters);
        double[] lanes =
                new double[] {
                    shouldFlip ? FieldUtil.applyX(x1) : x1,
                    shouldFlip ? FieldUtil.applyX(x2) : x2,
                    shouldFlip ? FieldUtil.applyX(x3) : x3
                };

        Logger.recordOutput("Detection/Lane1", lanes[0]);
        Logger.recordOutput("Detection/Lane2", lanes[1]);
        Logger.recordOutput("Detection/Lane3", lanes[2]);

        int bestIndex = -1;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i < lanes.length; i++) {
            double distance = Math.abs(laneX - lanes[i]);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = i;
            }
        }
        return bestIndex == -1 ? Optional.empty() : Optional.of(bestIndex);
    }

    /**
     * Creates a routine-bound trigger that rises once a running trajectory has exceeded the
     * allowable path error for long enough that the auto should fall back to retry pathfinding.
     */
    public static Trigger retryTrigger(AutoRoutine routine, AutoTrajectory trajectory) {
        Distance pathErrorTol = DriveConstants.ALLOWABLE_PATH_ERROR;
        return routine.observe(
                new BooleanSupplier() {
                    private final Timer errorCheckDelayTimer = new Timer();
                    private final Debouncer pathErrorDebouncer = new Debouncer(1.0);
                    private boolean wasActive = false;

                    @Override
                    public boolean getAsBoolean() {
                        boolean isActive = trajectory.active().getAsBoolean();

                        if (isActive && !wasActive) {
                            errorCheckDelayTimer.restart();
                        } else if (!isActive && wasActive) {
                            errorCheckDelayTimer.stop();
                            errorCheckDelayTimer.reset();
                            pathErrorDebouncer.calculate(false);
                        }

                        wasActive = isActive;

                        return isActive
                                && errorCheckDelayTimer.hasElapsed(2.0)
                                && (pathErrorDebouncer.calculate(
                                                robotState
                                                        .getActiveTrajectoryError()
                                                        .gte(pathErrorTol))
                                        || robotState.forcePathFind.get())
                                && robotState.getFieldRegion() == FieldRegion.NEUTRAL_ZONE;
                    }
                });
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
