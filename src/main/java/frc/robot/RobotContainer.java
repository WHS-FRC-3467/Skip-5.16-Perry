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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.DashboardBoolean;
import frc.lib.util.DashboardDouble;
import frc.lib.util.DashboardDoubleArray;
import frc.lib.util.FieldUtil;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.LoggedValue;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.autos.*;
import frc.robot.commands.autos.tuning.*;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.IntakeLinearConstants;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructureConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperstructureConstants;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.HubState;
import frc.robot.util.RobotSim;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.Set;

/**
 * Container class for the robot that holds all subsystems, controllers, and command bindings. This
 * class is responsible for:
 *
 * <ul>
 *   <li>Instantiating all subsystems
 *   <li>Configuring controller button bindings
 *   <li>Providing autonomous command selection
 *   <li>Setting up dashboard controls and telemetry
 * </ul>
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    final ShooterSuperstructure shooter;
    private final IntakeSuperstructure intake;
    private final Indexer indexer;
    private final Tower tower;
    // private final LEDs leds;
    // private final ObjectDetector objectDetector;

    // Reusable composite command to stop/stow/eject used in multiple bindings
    private final Command stopAllShooterAndRetract;

    // Controller
    private final CommandXboxControllerExtended controller =
            new CommandXboxControllerExtended(0).withDeadband(0.1);
    private final CommandXboxControllerExtended operatorController =
            new CommandXboxControllerExtended(1).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoOption> autoChooser;
    public final Field2d autoPreviewField = new Field2d();
    private Pose2d[] rawAutoPreviewPoses = new Pose2d[] {}; // Unflipped (blue-alliance) poses
    private Pose2d startPose = new Pose2d(); // Initialize start pose for auto dashboard tab
    // Change-only loggers for auto start pose checks
    private final LoggedValue<String> loggedStartPose;
    private final DashboardDouble dashboardInchesFromStart;
    private final DashboardBoolean dashboardPositionWithinStartTolerance;
    private final DashboardDouble dashboardDegreesFromStart;
    private final DashboardBoolean dashboardRotationWithinStartTolerance;
    private final DashboardDoubleArray dashboardStartPoseArray;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {

        drive = DriveConstants.get();
        shooter = ShooterSuperstructureConstants.get();
        intake = IntakeSuperstructureConstants.get();
        indexer = IndexerConstants.get();
        tower = TowerConstants.get();
        VisionConstants.create();
        // VisionOdometryCharacterizer.enable();
        // leds = LEDsConstants.get();
        // objectDetector = ObjectDetectorConstants.get();

        stopAllShooterAndRetract =
                Commands.parallel(
                        shooter.stopAndStow(),
                        indexer.stopCommand(),
                        tower.stopCommand(),
                        intake.extendIntake(),
                        shooter.retractHood());

        if (RobotBase.isSimulation()) {
            RobotSim.getInstance().addMechanismData(drive, shooter, indexer, intake);
        }
        AutoContext ctx =
                AutoContext.create(drive, intake, indexer, tower, shooter, Optional.empty());

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Initialize change-only loggers for start-pose checks
        this.loggedStartPose = new LoggedValue<>("Auto/StartPose");
        this.dashboardInchesFromStart = new DashboardDouble("Auto Pose Check/Inches from Start");
        this.dashboardPositionWithinStartTolerance =
                new DashboardBoolean("Auto Pose Check/Robot Position Within Tolerance");
        this.dashboardDegreesFromStart = new DashboardDouble("Auto Pose Check/Degrees from Start");
        this.dashboardRotationWithinStartTolerance =
                new DashboardBoolean("Auto Pose Check/Robot Rotation Within Tolerance");
        this.dashboardStartPoseArray = new DashboardDoubleArray("Start Pose (x, y, degrees)");

        // Default - No Auto
        autoChooser.addDefaultOption("None", NoneAuto.create());

        // Preload Autos
        autoChooser.addOption("PreloadAuto", PreloadAuto.create(ctx));

        // Neutral Autos
        // MLNeutralAuto.create(ctx, false, true)
        //         .ifPresent(a -> autoChooser.addOption("ML-Neutral-Safe-Left", a));

        // Citrus Autos
        C1678Auto.create(ctx, false, false)
                .ifPresent(a -> autoChooser.addOption("NeutralAuto-Left", a));
        C1678Auto.create(ctx, true, false)
                .ifPresent(a -> autoChooser.addOption("NeutralAuto-Right", a));

        // C1678Auto.create(ctx, false, true)
        //         .ifPresent(a -> autoChooser.addOption("NeutralAuto-Safe-Left", a));
        // C1678Auto.create(ctx, true, true)
        //         .ifPresent(a -> autoChooser.addOption("NeutralAuto-Safe-Right", a));

        // DepotAuto.create(ctx, false, false).ifPresent(a -> autoChooser.addOption("Depot", a));

        autoChooser.onChange(
                auto -> {
                    if (auto == null) {
                        rawAutoPreviewPoses = new Pose2d[] {};
                        autoPreviewField.getObject("path").setPoses(new Pose2d[] {});
                        return;
                    }
                    var pathPoses = auto.previewPoses().toArray(Pose2d[]::new);
                    if (pathPoses.length == 0) {
                        rawAutoPreviewPoses = new Pose2d[] {};
                        return;
                    }
                    pathPoses[0] = auto.startingPose();
                    rawAutoPreviewPoses = pathPoses;

                    // Apply alliance flip for initial preview
                    var flippedPoses =
                            Arrays.stream(rawAutoPreviewPoses)
                                    .map(FieldUtil::apply)
                                    .toArray(Pose2d[]::new);
                    autoPreviewField.getObject("path").setPoses(flippedPoses);

                    auto.command();
                });

        autoChooser.addOption(
                "Drive Wheel Radius Characterization", WheelCharacterizationAuto.create(ctx));

        // autoChooser.addOption(
        //         "Feedforward Characterization", FeedforwardCharacterizationAuto.create(ctx));

        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
    }

    /**
     * Configures button bindings for the Xbox controller. Maps controller inputs to robot commands
     * for teleop control.`
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        HubState hubState = HubState.getInstance();
        shooter.setDefaultCommand(
                Commands.either(
                        shooter.spinUpShooter(),
                        Commands.none(),
                        hubState.getEnablingSoon()
                                .or(hubState.getHubActive())
                                .or(DriverStation::isAutonomous)));

        Trigger readyToShootAtCurrentTarget =
                shooter.profileComplete.and(
                        robotState
                                .shouldFeed
                                .and(robotState.facingFeedTarget)
                                .or(robotState.shouldFeed.negate().and(robotState.facingTarget)));

        // Right Trigger: Shoot/Pass
        controller
                .rightTrigger()
                .whileTrue(
                        Commands.parallel(
                                Commands.either(
                                        DriveCommands.joystickDriveFacingFutureTarget(
                                                drive,
                                                () -> -controller.getLeftY() * 0.4,
                                                () -> -controller.getLeftX() * 0.4,
                                                robotState.feedLookaheadSeconds),
                                        DriveCommands.staticAimTowardsTarget(drive),
                                        robotState.shouldFeed),
                                shooter.shoot(),
                                Commands.sequence(
                                        Commands.waitSeconds(0.05),
                                        Commands.waitUntil(readyToShootAtCurrentTarget),
                                        Commands.parallel(indexer.shoot(), tower.shoot()))))
                .onFalse(stopAllShooterAndRetract);

        // Tap Right Bumper while Right Trigger held: Manually cycle intake
        controller
                .rightBumper()
                .and(controller.x().negate())
                .onTrue(Commands.defer(() -> intake.slowRetract(), Set.of(intake)))
                .onFalse(intake.stopRoller());

        // Tap Right Bumper while X held: Manually cycle intake within bumpers for hub shot
        controller
                .rightBumper()
                .and(controller.x())
                .onTrue(intake.hubShuffleStep())
                .onFalse(intake.stopRoller());

        // Left Trigger: Intake
        controller.leftTrigger().onTrue(intake.intake()).onFalse(intake.stopRoller());

        // D-Pad Up: Force Intake Linear Slide Back
        controller.leftBumper().onTrue(intake.retractIntake());

        // D-Pad Down: Unjam
        controller
                .povDown()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));

        // Driver X: Trench Shot (No-Vision Fallback)
        controller
                .x()
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.stopWithX(drive),
                                shooter.spinUpShooterToFixedDistance(
                                        FieldConstants.TRENCH_SHOT_DISTANCE),
                                Commands.parallel(indexer.shoot(), tower.shoot())))
                .onFalse(stopAllShooterAndRetract);

        // Driver Y: Midline Feed/Pass (No-Vision Fallback)
        controller
                .y()
                .whileTrue(
                        Commands.parallel(
                                shooter.spinUpShooterMidlineFeed(),
                                Commands.sequence(
                                        Commands.waitUntil(shooter.profileComplete),
                                        Commands.parallel(indexer.shoot(), tower.shoot()))))
                .onFalse(stopAllShooterAndRetract);

        // Driver A: Shot From Back of Robot Against Tower (No-Vision Fallback)
        controller
                .a()
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.stopWithX(drive),
                                shooter.spinUpShooterToFixedDistance(
                                        FieldConstants.Tower.TOWER_SHOT_DISTANCE),
                                Commands.parallel(indexer.shoot(), tower.shoot())))
                .onFalse(stopAllShooterAndRetract);

        controller
                .start()
                .and(controller.back())
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        robotState.resetPose(
                                                new Pose2d(
                                                        robotState
                                                                .getEstimatedPose()
                                                                .getTranslation(),
                                                        Rotation2d.kZero))));

        // Operator A: Home Hood and Intake
        operatorController
                .a()
                .whileTrue(Commands.parallel(intake.homeLinear(), shooter.homeHood()));

        // Operator B: Eject
        operatorController
                .b()
                .whileTrue(Commands.parallel(intake.ejectRoller(), indexer.eject(), tower.eject()));

        // Operator X: Retract Intake
        operatorController.x().onTrue(intake.retractIntake());

        // Operator POV Up: Trim shot power up
        operatorController.povUp().onTrue(shooter.trimFlywheelSpeedUp());

        // Operator POV Down: Trim shot power down
        operatorController.povDown().onTrue(shooter.trimFlywheelSpeedDown());

        // Operator Y: Manual Spinup
        operatorController
                .y()
                .whileTrue(shooter.shoot().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        controller
                .rightTrigger()
                .negate()
                .and(operatorController.y().negate())
                .onTrue(shooter.stopFlywheels());

        hubState.getEnablingSoon()
                .onTrue(
                        Commands.sequence(
                                Commands.sequence(
                                        controller.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5),
                                        controller.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5),
                                        controller.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5)),
                                Commands.sequence(
                                        operatorController.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5),
                                        operatorController.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5),
                                        operatorController.rumbleForTime(1.0, Seconds.of(0.5)),
                                        Commands.waitSeconds(0.5))));

        controller
                .joysticksZeroed
                .negate()
                .and(new Trigger(DriverStation::isDisabled))
                .whileTrue(controller.rumble(1.0).ignoringDisable(true));
    }

    /**
     * Initializes SmartDashboard with test commands and controls for subsystems. Adds commands to
     * the dashboard for manual testing and debugging.
     */
    private void initializeDashboard() {

        SmartDashboard.putData(
                "Home Intake and Shooter",
                Commands.parallel(intake.homeLinear(), shooter.homeHood()));

        // Intake Commands
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Intake", intake.intake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Retract", intake.retractIntake());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/Coast", intake.linearCoast());
        SmartDashboard.putData(IntakeLinearConstants.NAME + "/SlowRetract", intake.slowRetract());

        SmartDashboard.putData(
                "Debug/SetOdometryToTestPose",
                Commands.runOnce(() -> robotState.resetPose(new Pose2d(8, 5, Rotation2d.k180deg))));

        SmartDashboard.putData(
                "Fountain",
                Commands.sequence(
                                Commands.sequence(
                                        Commands.parallel(
                                                shooter.fountain(),
                                                indexer.fountain(),
                                                tower.fountain())),
                                Commands.parallel(shooter.idle(), indexer.idle(), tower.idle()))
                        .finallyDo(
                                () ->
                                        CommandScheduler.getInstance()
                                                .schedule(
                                                        shooter.stopAndStow(),
                                                        indexer.stopCommand(),
                                                        tower.stopCommand())));

        // Drivetrain Commands
        SmartDashboard.putData(
                "Drive to Start Pose",
                new DriveToPose(drive, () -> startPose)
                        .withDistanceTolerance(Meters.of(0.04))
                        .withAngularTolerance(Degrees.of(3)));
    }

    /**
     * Gets the selected autonomous command from the dashboard chooser.
     *
     * @return the autonomous command to run
     */
    public Command getAutonomousCommand() {
        AutoOption option = autoChooser.get();
        return option == null ? Commands.none() : option.command();
    }

    /**
     * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
     * path every 100ms.
     *
     * @param counter an integer from 0 to 4, that determines whether to spend the time to display
     *     info.
     */
    public void checkStartPose(int counter) {
        var pathObj = autoPreviewField.getObject("path");
        var poses = pathObj.getPoses();
        if (poses == null || poses.isEmpty()) {
            startPose = robotState.getEstimatedPose();
        } else {
            startPose = poses.get(0);
        }

        // Only update dashboard every 100 ms
        if (counter == 0) {
            /* Starting pose checker for auto */
            autoPreviewField.setRobotPose(robotState.getEstimatedPose());

            if (poses == null || poses.isEmpty()) {
                // No path defined; show defaults
                startPose = robotState.getEstimatedPose();
                SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
                SmartDashboard.putBoolean("Auto Pose Check/Robot Position Within Tolerance", false);
                SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
                SmartDashboard.putBoolean("Auto Pose Check/Robot Rotation Within Tolerance", false);
                return;
            }

            // Record start pose only when it changes to reduce NT traffic
            loggedStartPose.log(startPose.toString());
            autoPreviewField.getObject("startPose").setPose(startPose);

            Pose2d robotPose = robotState.getEstimatedPose();
            Distance distanceFromStartPose =
                    Meters.of(robotPose.getTranslation().getDistance(startPose.getTranslation()));
            double degreesFromStartPose =
                    Math.abs(robotPose.getRotation().minus(startPose.getRotation()).getDegrees());

            double[] startPoseArray = {
                startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees()
            };
            dashboardStartPoseArray.put(startPoseArray);

            dashboardInchesFromStart.put(
                    (int) Math.round(distanceFromStartPose.in(Inches) * 100.0) / 100.0);
            dashboardPositionWithinStartTolerance.put(
                    distanceFromStartPose.in(Inches)
                            < Constants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
            dashboardDegreesFromStart.put((int) Math.round(degreesFromStartPose * 100.0) / 100.0);
            dashboardRotationWithinStartTolerance.put(
                    degreesFromStartPose
                            < Constants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees));
        }
    }
}
