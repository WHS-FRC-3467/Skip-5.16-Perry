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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.AlwaysTunableNumber;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableBoolean;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.lib.util.PowerProfiler;
import frc.robot.RobotState;
import frc.robot.util.ShotTracker;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    /** Distance from hub in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap hubFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        hubFlywheelMap.put(1.8, 26.0);
        hubFlywheelMap.put(2.1, 27.0);
        hubFlywheelMap.put(2.5, 28.0);
        hubFlywheelMap.put(3.15, 31.5);
        hubFlywheelMap.put(3.55, 32.0);
        hubFlywheelMap.put(4.0, 33.5);
        hubFlywheelMap.put(4.5, 33.5);
        hubFlywheelMap.put(5.0, 35.5);
    }

    /** Distance from feed pose in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap feedFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        feedFlywheelMap.put(3.35, 25.0); // 32
        feedFlywheelMap.put(4.5, 32.0);
        feedFlywheelMap.put(5.5, 35.0);
        feedFlywheelMap.put(8.0, 37.0);
    }

    /** Distance from hub in meters -> hood angle in degrees */
    private static final InterpolatingDoubleTreeMap hubHoodMap = new InterpolatingDoubleTreeMap();

    static {
        hubHoodMap.put(1.8, 0.0);
        hubHoodMap.put(2.1, 4.5);
        hubHoodMap.put(2.51, 7.0);
        hubHoodMap.put(3.15, 8.67);
        hubHoodMap.put(3.55, 10.0);
        hubHoodMap.put(4.0, 13.0);
        hubHoodMap.put(4.5, 16.0);
        hubHoodMap.put(5.0, 21.0);
    }

    /** Distance from feed pose in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap feedHoodMap = new InterpolatingDoubleTreeMap();

    static {
        feedHoodMap.put(4.5, 24.0);
        feedHoodMap.put(5.0, 24.0);
        feedHoodMap.put(5.5, 27.0);
        feedHoodMap.put(7.0, 27.0);
        feedHoodMap.put(8.0, 27.0);
        feedHoodMap.put(9.0, 27.0);
        feedHoodMap.put(10.0, 27.0);
    }

    private final RobotState robotState = RobotState.getInstance();

    private final RotaryMechanism<?, ?> hoodIO;
    private final FlywheelMechanism<?> flywheelIO;

    private final Debouncer profileCompleteDebouncer = new Debouncer(0.1, DebounceType.kRising);
    public final LoggedTrigger profileComplete =
            new LoggedTrigger(
                    this.getName() + "/ProfileComplete",
                    () -> profileCompleteDebouncer.calculate(isProfileComplete()));

    private final LoggedTunableBoolean tuningMode =
            new LoggedTunableBoolean(getName() + "/Tuning/Enable", false);
    private final LoggedTunableNumber tuningFlywheelSpeedRPS =
            new LoggedTunableNumber(getName() + "/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodAngleDegrees =
            new LoggedTunableNumber(getName() + "/Tuning/HoodAngleDegrees", 0.0);

    // Default trim to apply
    private final AlwaysTunableNumber flywheelTrimDefaultRPS =
            new AlwaysTunableNumber(getName() + "/FlywheelTrimDefaultRPS", -1.0);
    // How much to add or subtract on each button press
    private final LoggedTunableNumber flywheelTrimStepRPS =
            new LoggedTunableNumber(getName() + "/FlywheelTrimStepRPS", 0.5);

    public final LoggedTrigger readyToShootAtCurrentTarget =
            new LoggedTrigger(
                    "RobotState/ReadyToShootAtCurrentTarget",
                    profileComplete.and(
                            robotState
                                    .shouldFeed
                                    .and(robotState.facingFeedTarget)
                                    .or(
                                            robotState
                                                    .shouldFeed
                                                    .negate()
                                                    .and(robotState.facingTarget))));

    // User-defined trim at runtime, not including default trim
    private AngularVelocity flywheelTrim = RotationsPerSecond.zero();

    // A dedicated utility helper to quantify shooter performance
    private ShotTracker shotTracker;

    /**
     * Trigger for whether we are at the static shooting state (robot steadily stationary, steadily
     * aligned to target, and shooter ready)
     */
    public final LoggedTrigger staticShotState =
            new LoggedTrigger(
                    getName() + "/StaticShotState",
                    () -> robotState.atStaticShootingPosition.and(profileComplete).getAsBoolean());

    /**
     * Gets the total flywheel trim to apply, including both default and user-defined runtime trim
     *
     * @return The total flywheel trim to apply
     */
    private AngularVelocity getFlywheelTrim() {
        return RotationsPerSecond.of(flywheelTrimDefaultRPS.get()).plus(flywheelTrim);
    }

    /**
     * Constructs a new ShooterSuperstructure subsystem with the specified hood and flywheel
     * mechanisms.
     *
     * @param hoodIO the hood mechanism for adjusting shot angle
     * @param flywheelIO the flywheel mechanism for spinning up shots
     */
    public ShooterSuperstructure(RotaryMechanism<?, ?> hoodIO, FlywheelMechanism<?> flywheelIO) {
        this.hoodIO = hoodIO;
        this.flywheelIO = flywheelIO;
        shotTracker = ShotTracker.create(this);
    }

    @Override
    public void periodic() {
        if (tuningMode.get()) {
            if (tuningMode.hasChanged(hashCode())
                    || tuningFlywheelSpeedRPS.hasChanged(hashCode())
                    || tuningHoodAngleDegrees.hasChanged(hashCode())) {
                setFlywheelVelocity(RotationsPerSecond.of(tuningFlywheelSpeedRPS.get()));
                setHoodPosition(Degrees.of(tuningHoodAngleDegrees.get()));
            }

            Logger.recordOutput(
                    getName() + "/Tuning/DistanceToTargetMeters",
                    robotState.getDistanceToTarget().in(Meters));
        }
        LoggerHelper.recordCurrentCommand(this.getName(), this);

        flywheelIO.periodic();
        hoodIO.periodic();

        // Centralize required polling for hopper detection and ball counting logic
        shotTracker.ballTrigger.getAsBoolean();
        robotState.hopperEmpty.getAsBoolean();

        Logger.recordOutput(getName() + "/TotalDrawWatts", getFlywheelPowerDraw().in(Watts));

        Logger.recordOutput(
                getName() + "/FlywheelTrimRPS", getFlywheelTrim().in(RotationsPerSecond));

        Logger.recordOutput(
                getName() + "/DesiredFlywheelLinearVelocityMPS",
                getDesiredFlywheelLinearVelocity().in(MetersPerSecond));
    }

    private void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelIO.runVelocity(
                velocity.plus(getFlywheelTrim()),
                FlywheelConstants.MAX_ACCELERATION,
                PIDSlot.SLOT_0);
    }

    private void setHoodPosition(Angle angle) {
        hoodIO.runUnprofiledPosition(angle, PIDSlot.SLOT_0);
    }

    private boolean isProfileComplete() {
        return flywheelIO
                .getVelocitySetpoint()
                .isNear(flywheelIO.getVelocityGoal(), RotationsPerSecond.of(0.2));
    }

    /**
     * Gets the current angle of the hood.
     *
     * @return the hood's current position angle
     */
    public Angle getHoodAngle() {
        return hoodIO.getPosition();
    }

    public LinearVelocity getLinearVelocity() {
        return flywheelIO.getLinearVelocity();
    }

    private AngularVelocity getFlywheelTrimStep() {
        return RotationsPerSecond.of(flywheelTrimStepRPS.get());
    }

    private AngularVelocity getDesiredFlywheelVelocity() {
        boolean shouldFeedNow = robotState.shouldFeed.getAsBoolean();
        InterpolatingDoubleTreeMap flywheelMap = shouldFeedNow ? feedFlywheelMap : hubFlywheelMap;

        Pose2d pose;
        if (shouldFeedNow) {
            pose = robotState.getFuturePose(robotState.feedLookaheadSeconds.get());
        } else {
            pose = robotState.getEstimatedPose();
        }

        return RotationsPerSecond.of(
                flywheelMap.get(robotState.getDistanceToTarget(pose.getTranslation()).in(Meters)));
    }

    public LinearVelocity getDesiredFlywheelLinearVelocity() {
        return MetersPerSecond.of(
                getDesiredFlywheelVelocity().in(RadiansPerSecond)
                        * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    private Angle getDesiredHoodAngle() {
        boolean shouldFeedNow = robotState.shouldFeed.getAsBoolean();
        InterpolatingDoubleTreeMap hoodMap = shouldFeedNow ? feedHoodMap : hubHoodMap;

        Pose2d pose;
        if (shouldFeedNow) {
            pose = robotState.getFuturePose(robotState.feedLookaheadSeconds.get());
        } else {
            pose = robotState.getEstimatedPose();
        }

        return Degrees.of(
                hoodMap.get(robotState.getDistanceToTarget(pose.getTranslation()).in(Meters)));
    }

    /**
     * Return the current power draw of the flywheel mechanism based on applied voltage and supply
     * current.
     */
    public Power getFlywheelPowerDraw() {
        return flywheelIO.getAppliedVoltage().times(flywheelIO.getSupplyCurrent());
    }

    /** Register the shooter with the power profiler. */
    public void registerMechanisms(PowerProfiler powerProfiler) {
        powerProfiler.registerMechanism(getName() + "/Flywheel", flywheelIO);
        powerProfiler.registerMechanism(getName() + "/Hood", hoodIO);
    }

    /**
     * Creates a command to set the hood to a specific angle.
     *
     * @param angle the target angle for the hood
     * @return command that sets the hood angle
     */
    public Command setHoodAngle(Angle angle) {
        return this.runOnce(() -> setHoodPosition(angle)).withName("Set Hood Angle");
    }

    /**
     * Creates a command to set the flywheel velocity (alternate spelling).
     *
     * @param velocity the target angular velocity for both flywheels
     * @return command that sets the flywheel speed
     */
    public Command setFlywheelSpeed(AngularVelocity velocity) {
        return this.runOnce(() -> setFlywheelVelocity(velocity)).withName("Set Flywheel Speed");
    }

    public Command coastFlywheels() {
        return this.runOnce(() -> flywheelIO.runCoast()).withName("Coast Flywheels");
    }

    private Command setShooterCommand(
            Supplier<AngularVelocity> flywheelVelocity, Supplier<Angle> hoodAngle, String name) {
        return Commands.run(
                        () -> {
                            setFlywheelVelocity(flywheelVelocity.get());
                            setHoodPosition(hoodAngle.get());
                        },
                        this)
                .withName(name);
    }

    public Command stopAndStow() {
        return Commands.sequence(setHoodAngle(Rotations.zero()), coastFlywheels())
                .withName("Stop and Stow Shooter");
    }

    /**
     * Prepares to shoot from a fixed distance by spinning up the flywheel and setting the hood
     * angle
     *
     * @return a Command to prepare for the fixed shot
     */
    public Command setShooterToFixedDistance(Distance distance, boolean isFeeding) {
        if (isFeeding) {
            return setShooterCommand(
                    () -> RotationsPerSecond.of(feedFlywheelMap.get(distance.in(Meters))),
                    () -> Degrees.of(feedHoodMap.get(distance.in(Meters))),
                    "Set Shooter to Fixed Feed Distance");

        } else {
            return setShooterCommand(
                    () -> RotationsPerSecond.of(hubFlywheelMap.get(distance.in(Meters))),
                    () -> Degrees.of(hubHoodMap.get(distance.in(Meters))),
                    "Set Shooter to Fixed Hub Distance");
        }
    }

    /**
     * Dynamically spins up the shooter to prepare for a shot based on the robot's current position
     * and target. Updates flywheel velocity and hood angle continuously based on distance to
     * target. This command runs perpetually and must be interrupted or timed out to stop.
     *
     * @return Command that continuously updates shooter parameters for the current target
     */
    public Command setShooterContinuous() {
        return setShooterCommand(
                this::getDesiredFlywheelVelocity, this::getDesiredHoodAngle, "Spin-Up Shooter");
    }

    public Command spinUpFlywheel() {
        return this.run(() -> setFlywheelVelocity(getDesiredFlywheelVelocity()))
                .withName("Spin-Up Flywheel");
    }

    public Command fountain() {
        return Commands.sequence(
                setHoodAngle(Degrees.of(24.0)), setFlywheelSpeed(RotationsPerSecond.of(10.0)));
    }

    public Command homeHood() {
        return Commands.sequence(this.runOnce(() -> hoodIO.runDutyCycle(-0.1, true)), this.idle())
                .finallyDo(
                        () -> {
                            hoodIO.setEncoderPosition(Rotations.zero());
                            hoodIO.runBrake();
                        })
                .withName("Home Hood");
    }

    /**
     * A blocking command that lowers the hood and waits until it is zeroed. This should be included
     * with a timeout to compensate for possible sensor error. Primarily for use in autos.
     */
    public Command retractHood() {
        return setHoodAngle(Rotations.zero())
                .andThen(
                        Commands.waitUntil(
                                () -> hoodIO.getPosition().lte(HoodConstants.TOLERANCE)));
    }

    public Command trimFlywheelSpeedUp() {
        // Doesn't require subsystem to allow for trimming while shooting
        return Commands.runOnce(() -> flywheelTrim = flywheelTrim.plus(getFlywheelTrimStep()));
    }

    public Command trimFlywheelSpeedDown() {
        // Doesn't require subsystem to allow for trimming while shooting
        return Commands.runOnce(() -> flywheelTrim = flywheelTrim.minus(getFlywheelTrimStep()));
    }

    /** Closes all underlying mechanisms and releases resources. */
    @Override
    public void close() {
        flywheelIO.close();
        hoodIO.close();
    }
}
