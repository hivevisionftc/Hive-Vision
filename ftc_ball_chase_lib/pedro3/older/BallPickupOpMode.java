/*
 * Limelight 3A (SSD neural detector) -> BallMath -> follower pickup example.
 * Migrated from Pedro 1.x -> 2.1.2 -> Pedro Pathing 3.x (Sept 2026).
 *
 * TeleOp: every loop it asks the Limelight 3A for the best ball's tx/ty/ta,
 * turns that into a FIELD (x, y) for the ball with BallMath, computes where the
 * robot CENTER must be so the intake mouth lands on the ball (pickup pose), and
 * drives a path to it. When the robot is within ARRIVE_DIST it
 * holds position and reports "pickup ready".
 *
 * 2.1.2 -> 3.x changes:
 *   FollowerBuilder / *Constants   -> Constants.createFollower(hardwareMap)
 *                                        (older/pedroPathing/Constants.java,
 *                                        Configuration/ConfigVar-based; TUNE ME)
 *   com.pedropathing.geometry.Pose  -> com.pedropathing.math.Pose
 *   pathBuilder()/PathChain/BezierLine -> Paths.line(...) / Path + path.linear(...)
 *   follower.followPath(path)       -> follower.follow(path)
 *   follower.setStartingPose()      -> follower.setPose()
 *   follower.holdPoint(p, false)    -> follower.hold(p, false)
 *
 * Uses the FTC-native Limelight hardware class (no third-party helper needed):
 *   com.qualcomm.hardware.limelightvision.Limelight3A / LLResult
 * (ship with the FTC SDK - see FtcRobotController's SensorLimelight3A sample),
 * plus BallMath.java (this folder) and the pathing library.
 *
 * Calibrate the constants at the top before first run (see earlier README):
 *   CAM_H / BALL_H / CAM_PITCH / INTAKE_REACH / TA_CALIB_K
 *
 * Notes / limits:
 *   - Uses the PRIMARY target (best NN detection) only: tx/ty/ta describe the
 *     single strongest ball. The 3A emits up to 10 detections; to collect
 *     several balls you must iterate result.getDetectorResults() and pick the
 *     next one instead of using getTx()/getTy()/getTa().
 *   - tx/ty are the pinhole/tangent angles from the optical axis (deg).
 *     ty must be POSITIVE (ball above the image center), which happens
 *     automatically with a downward CAM_PITCH - a level camera cannot see a
 *     ground ball and BallMath.ballFieldPos() returns null.
 *   - ta is the target's 0..1 image fraction; below MIN_TA we treat it as
 *     "no target" instead of chasing noise.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Limelight Ball Pickup (Pathing)", group = "Hive Vision")
public class BallPickupOpMode extends LinearOpMode {

    /* ---- calibration (change per robot / field / camera mount) ---- */
    private static final String LIMELIGHT_NAME = "limelight";  // hardware-map name of the 3A
    private static final double CAM_H        = 12.0;   // camera optical center height, inches
    private static final double BALL_H       = 3.0;    // ball CENTER height (ball radius), inches
    private static final double CAM_PITCH    = 25.0;   // optical axis angle BELOW horizontal, deg
    private static final double INTAKE_REACH = 12.0;   // robot center -> intake mouth, inches
    private static final double MIN_TA       = 0.005;  // ignore nothing-smaller-than-this (0..1)
    private static final double ARRIVE_DIST  = 3.0;    // stop this many inches from the pose
    private static final double RE_FOLLOW_DIST = 2.0;  // rebuild path when error exceeds this
    private static final double MAX_CHASE_DIST = 120.0; // don't chase a ball 10 ft "away"
    private static final double TA_CALIB_K   = 120.0;  // dist = sqrt(K/ta); calibrate per earlier README

    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);   // how often we ask the 3A for data
        limelight.start();              // must be called or getLatestResult() returns null

        follower = Constants.createFollower(hardwareMap);
        // Set this to the robot's actual start pose. The example reads robot
        // x/y/heading back from follower.pose() every frame, so a working
        // localizer + drive/turn constants must be present.
        follower.setPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            driveToBall();
            telemetry.addData("pose", "%.1f, %.1f @ %.1f deg",
                    follower.pose().x(),
                    follower.pose().y(),
                    Math.toDegrees(follower.pose().heading()));
            telemetry.addData("mode", follower.isBusy() ? "following" : "held");
            telemetry.update();
        }

        limelight.stop();
    }

    /** Ask the Limelight for the best ball, then path the robot to the pickup pose. */
    private void driveToBall() {
        Pose robotPose = follower.pose();
        double rx = robotPose.x();
        double ry = robotPose.y();
        double robotHeadingDeg = Math.toDegrees(robotPose.heading());

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("ball", "no limelight result");
            return;
        }
        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();
        if (ta < MIN_TA) {
            telemetry.addData("ball", "no target (ta=%.4f)", ta);
            return;
        }

        /* BallMath: one ray + the ball's known height -> field position. */
        double[] ball = BallMath.ballFieldPos(rx, ry, robotHeadingDeg,
                tx, ty, CAM_PITCH, CAM_H, BALL_H);
        if (ball == null) {
            // Ray never reaches the ball's height (camera too level / ty not
            // positive enough). Fall back to angular-size distance only.
            Double dist = BallMath.rangeFromTa(ta, TA_CALIB_K);
            if (dist == null || dist > MAX_CHASE_DIST) {
                telemetry.addData("ball", "ty too low; tilt camera down");
                return;
            }
            // Approximate heading: robot yaw + tx, good enough when ty is small.
            double approxBearing = robotHeadingDeg + tx;
            double bx = rx + dist * Math.sin(Math.toRadians(approxBearing));
            double by = ry + dist * Math.cos(Math.toRadians(approxBearing));
            telemetry.addData("ta fallback", "%.1f in @ %+.1f deg", dist, approxBearing);
            goTo(bx, by, Math.toRadians(approxBearing), robotPose, rx, ry);
            return;
        }

        double bx = ball[0];
        double by = ball[1];
        double ballDist = ball[2];
        double bearingDeg = ball[3];

        /* Where the robot CENTER must be so the intake mouth sits on the ball. */
        double[] pickup = BallMath.pickupPose(bx, by, bearingDeg, INTAKE_REACH);
        double px = pickup[0];
        double py = pickup[1];
        double targetHeadingRad = Math.toRadians(pickup[2]);

        telemetry.addData("ball", "x=%.1f y=%.1f dist=%.1f bearing=%+.1f",
                bx, by, ballDist, bearingDeg);
        telemetry.addData("pickup pose", "x=%.1f y=%.1f head=%+.1f",
                px, py, pickup[2]);

        if (ballDist > MAX_CHASE_DIST) {
            telemetry.addData("drive", "too far to chase");
            return;
        }
        goTo(px, py, targetHeadingRad, robotPose, rx, ry);
    }

    /** Drive the robot center toward (tx, ty) with a path whose end heading
     *  is targetHeadingRad. Rebuilds the path only when the remaining error is
     *  too big so the robot doesn't thrash at the arrival point. */
    private void goTo(double tx, double ty, double targetHeadingRad, Pose robotPose,
                      double rx, double ry) {
        double error = Math.hypot(tx - rx, ty - ry);
        if (error <= ARRIVE_DIST) {
            if (!follower.isBusy()) {
                follower.hold(new Pose(tx, ty, targetHeadingRad), false);
                telemetry.addData("drive", "pickup READY (%.1f in)", error);
            }
            return;
        }
        if (error > RE_FOLLOW_DIST || !follower.isBusy()) {
            Path path = Paths.line(new Pose(rx, ry), new Pose(tx, ty));
            // End facing the ball so the intake mouth is the leading edge.
            path.linear(robotPose.heading(), targetHeadingRad);
            follower.follow(path);
            telemetry.addData("drive", "%.1f in to pose", error);
        }
    }
}