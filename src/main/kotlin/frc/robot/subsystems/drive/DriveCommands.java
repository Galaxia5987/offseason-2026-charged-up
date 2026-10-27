// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.Drive.DRIVE_BASE_RADIUS;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Second;

import com.pathplanner.lib.config.PIDConstants;
import frc.robot.InitializerKt;
import frc.robot.lib.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;

public class DriveCommands {
    private static final Drive drive = InitializerKt.getDrive();

    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 3; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final double accelerationLimitShootOnMove = 2.2; // m/s^2

    private static final PIDConstants angleGains = new PIDConstants(5.0, 0.0, 0.0);
    private static final TrapezoidProfile.Constraints angleConstraints =
            new TrapezoidProfile.Constraints(100000, 100000);

    private static final SlewRateLimiter slewRateLimiterX =
            new SlewRateLimiter(accelerationLimitShootOnMove);
    private static final SlewRateLimiter slewRateLimiterY =
            new SlewRateLimiter(accelerationLimitShootOnMove);

    private static Rotation2d lastRotationSetpoint = Rotation2d.kZero;

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    public static Command resetGyro() {
        return drive.run(
                        (_) -> {
                            Angle resetHeading =
                                    AllianceHelperKt.getIS_RED() ? Degrees.of(180) : Degrees.zero();
                            drive.resetOdometry(
                                    new Pose2d(
                                            drive.getPose().getTranslation(),
                                            new Rotation2d(resetHeading)));
                            drive.resetGyro(resetHeading);
                        })
                .named("DriveCommands#resetGyro"); // TODO: Might cause problems with disabled
    }

    public static Command resetByPoseEstimation() {
        return drive.run(
                        (_) -> {
                            drive.resetOdometry(
                                    BetterPoseEstimator.getInstance().getEstimatedPose());
                            drive.resetGyro(
                                    BetterPoseEstimator.getInstance()
                                            .getEstimatedPose()
                                            .getRotation()
                                            .getMeasure());
                        })
                .named("DriveCommands#resetByPoseEstimation");
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return drive.run(
                        (coroutine) -> {
                            while (true) {
                                // Get linear velocity
                                Translation2d linearVelocity =
                                        getLinearVelocityFromJoysticks(
                                                xSupplier.getAsDouble(), ySupplier.getAsDouble());

                                // Apply rotation deadband
                                double omega =
                                        MathUtil.applyDeadband(
                                                omegaSupplier.getAsDouble(), DEADBAND);

                                // Square rotation value for more precise control
                                omega = Math.copySign(omega * omega, omega);

                                double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
                                double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec();

                                // Convert to field relative speeds & send command
                                ChassisVelocities speeds =
                                        new ChassisVelocities(
                                                linearVelocity.getX() * maxSpeed,
                                                linearVelocity.getY() * maxSpeed,
                                                omega * maxAngularSpeed);
                                boolean isFlipped = AllianceHelperKt.getIS_RED();

                                ChassisVelocities robotRelativeSpeeds =
                                        speeds.toRobotRelative(
                                                isFlipped
                                                        ? drive.getRotation()
                                                                .plus(Rotation2d.fromDegrees(180))
                                                        : drive.getRotation());

                                drive.runVelocity(robotRelativeSpeeds);
                                coroutine.yield();
                            }
                        }).withPriority(Command.LOWEST_PRIORITY)
                .named("DriveCommands#JoystickDrive");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        angleGains.kP, angleGains.kI, angleGains.kD, angleConstraints);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return drive.run(
                        (coroutine) -> {
                            angleController.reset(drive.getRotation().getRadians());
                            while (true) {
                                // Get linear velocity
                                Translation2d linearVelocity =
                                        getLinearVelocityFromJoysticks(
                                                xSupplier.getAsDouble(), ySupplier.getAsDouble());
                                double omega;

                                double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
                                double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec();

                                omega =
                                        MathUtil.applyDeadband(
                                                omegaSupplier.getAsDouble(), DEADBAND);

                                // Square rotation value for more precise control
                                omega = Math.copySign(omega * omega, omega) * maxAngularSpeed;
                                // Convert to field relative speeds & send command
                                ChassisVelocities speeds =
                                        new ChassisVelocities(
                                                linearVelocity.getX() * maxSpeed,
                                                linearVelocity.getY() * maxSpeed,
                                                omega);
                                boolean isFlipped = AllianceHelperKt.getIS_RED();
                                drive.runVelocity(
                                        speeds.toRobotRelative(
                                                isFlipped
                                                        ? drive.getRotation()
                                                                .plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()));
                                coroutine.yield();
                            }
                        })
                .named("DriveCommands#joystickDriveAtAngle");
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Command.noRequirements(
                        (_) -> {
                            // Reset data
                            velocitySamples.clear();
                            voltageSamples.clear();
                        })
                .named("ResetSamples")
                .andThen(
                        Command.race(
                                        drive.run(
                                                        (coroutine) -> {
                                                            // Allow modules to orient
                                                            while (true) {
                                                                drive.runCharacterization(0.0);
                                                                coroutine.yield();
                                                            }
                                                        })
                                                .named("stopMotors"),
                                        Command.waitFor(Second.of(FF_START_DELAY))
                                                .named("stopMotorsTimeout"))
                                .withAutomaticName())
                .andThen(
                        drive.run(
                                        (coroutine) -> {
                                            // Accelerate and gather data
                                            timer.restart();
                                            while (true) {
                                                double voltage = timer.get() * FF_RAMP_RATE;
                                                drive.runCharacterization(voltage);
                                                velocitySamples.add(
                                                        drive.getFFCharacterizationVelocity());
                                                voltageSamples.add(voltage);
                                                coroutine.yield();
                                            }
                                        })
                                .named("CollectFFData"))
                .andThen(
                        Command.noRequirements(
                                        (coroutine) -> {
                                            int n = velocitySamples.size();
                                            double sumX = 0.0;
                                            double sumY = 0.0;
                                            double sumXY = 0.0;
                                            double sumX2 = 0.0;
                                            for (int i = 0; i < n; i++) {
                                                sumX += velocitySamples.get(i);
                                                sumY += voltageSamples.get(i);
                                                sumXY +=
                                                        velocitySamples.get(i)
                                                                * voltageSamples.get(i);
                                                sumX2 +=
                                                        velocitySamples.get(i)
                                                                * velocitySamples.get(i);
                                            }
                                            double kS =
                                                    (sumY * sumX2 - sumX * sumXY)
                                                            / (n * sumX2 - sumX * sumX);
                                            double kV =
                                                    (n * sumXY - sumX * sumY)
                                                            / (n * sumX2 - sumX * sumX);

                                            NumberFormat formatter = new DecimalFormat("#0.00000");
                                            System.out.println(
                                                    "********** Drive FF Characterization Results **********");
                                            System.out.println("\tkS: " + formatter.format(kS));
                                            System.out.println("\tkV: " + formatter.format(kV));
                                        })
                                .named("FFCharacterization"))
                .withAutomaticName();
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Command.parallel(
                        // Drive control sequence
                        drive.run(
                                        (coroutine) -> {
                                            // Reset acceleration limiter
                                            limiter.reset(0.0);
                                            // Turn in place, accelerating up to full speed
                                            while (true) {
                                                double speed =
                                                        limiter.calculate(
                                                                WHEEL_RADIUS_MAX_VELOCITY);
                                                drive.runVelocity(
                                                        new ChassisVelocities(0.0, 0.0, speed));
                                                coroutine.yield();
                                            }
                                        })
                                .named("RunVelocity"),
                        // Measurement sequence
                        Command.sequence(
                                        // Wait for modules to fully orient before starting
                                        // measurement
                                        Command.waitFor(Second.of(1.0)).named("WaitOneSecond"),

                                        // Record starting measurement
                                        Command.noRequirements(
                                                        (coroutine) -> {
                                                            state.positions =
                                                                    drive
                                                                            .getWheelRadiusCharacterizationPositions();
                                                            state.lastAngle = drive.getRotation();
                                                            state.gyroDelta = 0.0;
                                                            while (true) {
                                                                // Update gyro delta
                                                                var rotation = drive.getRotation();
                                                                state.gyroDelta +=
                                                                        Math.abs(
                                                                                rotation.minus(
                                                                                                state.lastAngle)
                                                                                        .getRadians());
                                                                state.lastAngle = rotation;
                                                                coroutine.yield();
                                                            }
                                                        })
                                                .named("RecordGyro"),
                                        Command.noRequirements(
                                                        (_) -> {
                                                            // When cancelled, calculate and print
                                                            // results
                                                            double[] positions =
                                                                    drive
                                                                            .getWheelRadiusCharacterizationPositions();
                                                            double wheelDelta = 0.0;
                                                            for (int i = 0; i < 4; i++) {
                                                                wheelDelta +=
                                                                        Math.abs(
                                                                                        positions[i]
                                                                                                - state.positions[
                                                                                                        i])
                                                                                / 4.0;
                                                            }
                                                            double wheelRadius =
                                                                    (state.gyroDelta
                                                                                    * DRIVE_BASE_RADIUS)
                                                                            / wheelDelta;

                                                            NumberFormat formatter =
                                                                    new DecimalFormat("#0.000");
                                                            System.out.println(
                                                                    "********** Wheel Radius Characterization Results **********");
                                                            System.out.println(
                                                                    "\tWheel Delta: "
                                                                            + formatter.format(
                                                                                    wheelDelta)
                                                                            + " radians");
                                                            System.out.println(
                                                                    "\tGyro Delta: "
                                                                            + formatter.format(
                                                                                    state.gyroDelta)
                                                                            + " radians");
                                                            System.out.println(
                                                                    "\tWheel Radius: "
                                                                            + formatter.format(
                                                                                    wheelRadius)
                                                                            + " meters, "
                                                                            + formatter.format(
                                                                                    Units
                                                                                            .metersToInches(
                                                                                                    wheelRadius))
                                                                            + " inches");
                                                        })
                                                .named("Characterize"))
                                .withAutomaticName())
                .withAutomaticName();
    }

    public static Command stop() {
        return drive.run((_) -> drive.stop()).named("stopDrive");
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
