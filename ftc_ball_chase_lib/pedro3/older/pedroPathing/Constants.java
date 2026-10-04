/*
 * Constants - Pedro Pathing 3.x follower construction.
 *
 * Pedro 3 dropped the 2.x `FollowerBuilder` + `*Constants` object graph entirely.
 * A follower is now assembled directly from three parts:
 *
 *     Follower(localizer, drivetrain, algorithm)
 *
 * Each part is built from a `ConfigVar` bag via a `Configuration<T>` lambda:
 *
 *     MecanumConfig cfg = new MecanumConfig(c -> { c.frontLeftName.set("leftFront"); ... });
 *     Mecanum drivetrain = new Mecanum(hardwareMap, cfg);
 *
 * CAUTION: every number/name below is a placeholder so the file COMPILES.
 * TUNE ME for the actual robot before trusting it on the field - motor names,
 * IMU/encoder names, pod offsets and Foresight coefficients.
 */
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Constants {

    /* ---- TUNE ME: match your robot config ---- */
    public static final String LEFT_FRONT = "leftFront";   // also the left encoder
    public static final String LEFT_REAR  = "leftRear";
    public static final String RIGHT_FRONT = "rightFront"; // also the right encoder
    public static final String RIGHT_REAR  = "rightRear";  // also the strafe encoder
    public static final String IMU_NAME    = "imu";

    /* ---- TUNE ME: drivetrain speed limits (in/s) ---- */
    public static final double MAX_FORWARD_VELOCITY = 50.0;
    public static final double MAX_STRAFE_VELOCITY  = 50.0;

    /* ---- TUNE ME: how fast the robot can shed speed (in/s^2) ---- */
    public static final double NATURAL_FORWARD_DECELERATION = 60.0;
    public static final double NATURAL_STRAFE_DECELERATION  = 60.0;

    private Constants() {
    }

    /** Build a Pedro 3 Follower from the hardware map (placeholder values). */
    public static Follower createFollower(HardwareMap hardwareMap) {
        MecanumConfig mecanumConfig = new MecanumConfig(c -> {
            c.frontLeftName.set(LEFT_FRONT);
            c.backLeftName.set(LEFT_REAR);
            c.frontRightName.set(RIGHT_FRONT);
            c.backRightName.set(RIGHT_REAR);

            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        });

        ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {
            c.leftEncoderName.set(LEFT_FRONT);
            c.rightEncoderName.set(RIGHT_FRONT);
            c.strafeEncoderName.set(RIGHT_REAR);
            c.imuName.set(IMU_NAME);

            // TUNE ME: pod offsets in inches
            c.leftPodY.set(5.0);
            c.rightPodY.set(-5.0);
            c.strafePodX.set(2.0);

            // TUNE ME: ticks -> inches / radians
            c.forwardTicksToInches.set(0.05);
            c.strafeTicksToInches.set(0.05);
            c.turnTicksToRadians.set(0.05);

            c.leftEncoderDirection.set(1.0);
            c.rightEncoderDirection.set(-1.0);
            c.strafeEncoderDirection.set(-1.0);

            // TUNE ME: hub orientation
            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        });

        ForesightConfig foresightConfig = new ForesightConfig(c -> {
            // TUNE ME: Foresight gains
            c.headingFeedback.set(Controller.pid(0.6, 0.0, 0.015));
            c.headingStaticFF.set(Controller.staticFeedforward(0.0));
            c.forwardTranslational.set(Controller.pid(0.1, 0.0, 0.0));
            c.strafeTranslational.set(Controller.pid(0.1, 0.0, 0.0));
            c.brake.set(Controller.pid(0.025, 0.0, 0.0));
            c.coast.set(Controller.pid(0.025, 0.0, 0.0));

            // TUNE ME: Foresight braking. linear acts on v*|v|, quadratic on v*|v|^2.
            c.linearBrakeCoefficients.set(Matrix.diag(Vector2D.cartesian(0.005, 0.005)));
            c.quadraticBrakeCoefficients.set(Matrix.diag(Vector2D.cartesian(0.0005, 0.0005)));
            // x = quadratic (omega^2) coefficient, y = linear (omega) coefficient
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.004, 0.02));

            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);
        });

        Mecanum drivetrain = new Mecanum(hardwareMap, mecanumConfig);
        ThreeWheelIMULocalizer localizer = new ThreeWheelIMULocalizer(hardwareMap, localizerConfig);
        Foresight algorithm = new Foresight(foresightConfig);

        return new Follower(localizer, drivetrain, algorithm);
    }

    /** Pedro 3 Quickstart-style alias. */
    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }
}