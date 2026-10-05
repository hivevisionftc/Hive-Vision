/*
 * ConstantsPinpoint - same Follower as Constants.java, with Pinpoint odometry
 * swapped in for the three-wheel+IMU localizer.
 *
 * Use this INSTEAD of Constants.java when the robot carries a goBILDA Pinpoint.
 * Everything above the localizer block (mecanum directions, Foresight gains) is
 * unchanged, because a Pinpoint replaces the localizer only - it is not a
 * drivetrain and not an algorithm.
 *
 * Requires Pedro 3 revhub jars: pedro3_core.jar + pedro3_revhub.jar.
 * (Constants.java's sibling in limelight_3a_pedro_example targets Pedro 2.x,
 *  where the same job is done by PinpointConstants + FollowerBuilder. The two
 *  APIs spell the same physics differently - see the note at the bottom.)
 *
 * CAUTION: every number/name below is a placeholder so the file COMPILES.
 * TUNE ME before trusting it on the field.
 *
 * TUNE IN THIS ORDER - the first three dominate every other error:
 *   1. ticksPerUnit      measure real travel, divide by ticks read from telemetry
 *   2. yPodOffset/xPodOffset   ruler, not guesswork (wrong pod offsets look like
 *                        a bad heading gain and send you chasing yawScalar)
 *   3. yPodDirection/xPodDirection  if the pose mirrors or yaws the wrong way
 *                        on a pure forward push, flip these - do NOT touch scales
 *   4. yawScalar         LAST, and only for consistent heading drift.
 *                        Start 1.0, step +/-0.0005, keep |scalar - 1| < 0.02
 *
 * NOTE: there is no camera pose here on purpose. Pedro takes heading from the
 * Pinpoint's internal IMU and ignores the camera pose entirely. The camera mount
 * (CAM_PITCH_DEG, CAM_H, CAM_X_OFFSET, CAM_Y_OFFSET) stays in HiveConfig, because
 * that is what BallTracker uses to turn tx/ty into a field position.
 *
 * After init, call follower.setStartPose(...) / localizer.resetPosAndIMU(...) at
 * the top of an OpMode rather than trusting a pre-calibrated IMU. Never
 * recalibrate mid-match - a recalibrate mid-run throws away your accumulated
 * odometry and the pose jumps.
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
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.OptionalDouble;

public final class ConstantsPinpoint {

    /* ---- TUNE ME: match your robot config ---- */
    public static final String LEFT_FRONT = "leftFront";
    public static final String LEFT_REAR  = "leftRear";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String RIGHT_REAR  = "rightRear";
    public static final String PINPOINT_NAME = "pinpoint";

    /* ---- TUNE ME: drivetrain speed limits (in/s) ---- */
    public static final double MAX_FORWARD_VELOCITY = 50.0;
    public static final double MAX_STRAFE_VELOCITY  = 50.0;

    /* ---- TUNE ME: how fast the robot can shed speed (in/s^2) ---- */
    public static final double NATURAL_FORWARD_DECELERATION = 60.0;
    public static final double NATURAL_STRAFE_DECELERATION  = 60.0;

    /* ---- TUNE ME: Pinpoint physical mounting, in inches ---- */
    // yPod = the pod pair that tracks FORWARD travel.
    // xPod = the single pod that tracks STRAFE travel.
    // Both are offsets FROM ROBOT CENTER, so signs matter: +y is LEFT,
    // +x is FORWARD. A goBILDA 4-bar pod is usually mounted with its center
    // roughly on the centerline; x is whatever your bracket gives you.
    public static final double PINPOINT_Y_POD_OFFSET = 0.0;
    public static final double PINPOINT_X_POD_OFFSET = 0.0;

    /* ---- TUNE ME: encoder scale, in ticks per inch ---- */
    // Spec sheet says 8192 ticks/in, but measure YOUR pod and divide:
    //   inchesDriven / ticksReported. Aim to land within 2% of spec.
    public static final double PINPOINT_TICKS_PER_UNIT = 8192.0;

    /* ---- TUNE ME: encoder directions ---- */
    // Only used by a manual update path; the Pinpoint auto-detects its own
    // directions from the wiring when both are FORWARD. Set these only if you
    // are feeding odometry in yourself.
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /* ---- TUNE ME: pod type ---- */
    // goBILDA_4_BAR_POD for the standard 4-bar pods. Use the external/3-wire
    // option ONLY if you have run the pods through an external encoder port.
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /* ---- TUNE ME: heading gain ---- */
    // 1.0 is nominal. Typical good values land 1.0005-1.0010. If your robot
    // consistently over- or under-rotates on a pure spin, nudge this - but fix
    // ticksPerUnit and the pod offsets FIRST, because both masquerade as this.
    public static final double PINPOINT_YAW_SCALAR = 1.0;

    private ConstantsPinpoint() {
    }

    /** Build a Pedro 3 Follower with Pinpoint odometry (placeholder values). */
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

        PinpointConfig localizerConfig = new PinpointConfig(c -> {
            c.name.set(PINPOINT_NAME);

            // pod offsets, inches, relative to robot center
            c.yPodOffset.set(PINPOINT_Y_POD_OFFSET);
            c.xPodOffset.set(PINPOINT_X_POD_OFFSET);

            // encoder scale + the unit all three of those are expressed in.
            // ticksPerUnit is an OptionalDouble here, so it has to be wrapped -
            // plain 8192.0 will not compile.
            c.ticksPerUnit.set(OptionalDouble.of(PINPOINT_TICKS_PER_UNIT));
            c.encoderResolutionUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);

            c.yPodDirection.set(PINPOINT_Y_POD_DIRECTION);
            c.xPodDirection.set(PINPOINT_X_POD_DIRECTION);
            c.podType.set(PINPOINT_POD_TYPE);
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
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        Foresight algorithm = new Foresight(foresightConfig);

        return new Follower(localizer, drivetrain, algorithm);
    }

    /** Pedro 3 Quickstart-style alias. */
    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }

    /* ------------------------------------------------------------------
     * Pedro 2.x equivalent, for the jars in limelight_3a_pedro_example.
     * Same four physical numbers, different spelling:
     *
     *   PinpointConstants localizer = new PinpointConstants()
     *           .hardwareMapName("pinpoint")
     *           .forwardPodY(PINPOINT_Y_POD_OFFSET)   // forward pod  <-> yPodOffset
     *           .strafePodX(PINPOINT_X_POD_OFFSET)    // strafe pod   <-> xPodOffset
     *           .forwardEncoderDirection(EncoderDirection.FORWARD)
     *           .strafeEncoderDirection(EncoderDirection.FORWARD)
     *           .encoderResolution(0.01)              // INCHES per tick, NOT ticks per inch
     *           .distanceUnit(DistanceUnit.INCH);
     *
     *   return new FollowerBuilder(followerConstants, hardwareMap)
     *           .mecanumDrivetrain(mecanum)
     *           .pinpointLocalizer(localizer)
     *           .build();
     *
     * Watch the inversion: Pedro 3 wants TICKS PER INCH (8192), Pedro 2.x wants
     * INCHES PER TICK (0.000122). Copying the number across verbatim gives you a
     * 67000x scale error, which reads as "the pose exploded".
     *
     * Pedro 2.x PinpointConstants has no yawScalar - the goBILDA driver owns
     * that internally there, so drop the yawScalar line rather than hunting for it.
     * ------------------------------------------------------------------ */
}
