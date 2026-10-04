/*
 * MecanumWrangler - BallWrangler backed by raw mecanum motor output (no Pedro,
 * no odometry requirement). All camera-relative verbs (find / grab / scan /
 * approach / align / backOff / nudge) run off the Limelight ray alone, exactly
 * like BallChaseController.
 *
 * Pose verbs (searchAt / then / thenReturnTo) need localization: pass a
 * PoseRouter (GoBilda Pinpoint, dead wheels, IMU+encoders ...). Without one
 * they fail fast and the chain moves on, so a pure camera-relative auto still
 * works.
 *
 * scan() needs a heading. A HeadingSource (or a PoseRouter that reports one)
 * gives a real reading; otherwise the drift is estimated from drive time via
 * TURN_RATE_RAD_PER_POWER_SEC - tune it, or accept coarse sweeps.
 */
package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BallTracker;

public class MecanumWrangler extends BallWrangler {

    /** Plug in any localization (Pinpoint, dead wheels, encoders + IMU).
     *  Null = pose verbs fail fast; camera-relative verbs are unaffected.
     *  update() is polled once per go() loop (and from loopHook() when you own
     *  the loop) for routers that need periodic reads (Pinpoint poll, a go-to
     *  PID tick); it is a no-op by default. */
    public interface PoseRouter {
        RobotPose getPose();
        void startGoTo(RobotPose target, double speedPower);
        boolean isBusy();
        void stop();
        default void update() {}
    }

    /** Heading source for scan()/turn sweeps. Heading must be CCW+ radians
     *  standard. Null falls back to a drive-time estimate. */
    public interface HeadingSource {
        double headingRad();
    }

    public static double TURN_RATE_RAD_PER_POWER_SEC = 3.5;  // dead-reckon scan estimate
    public static double GO_TO_POWER = 0.7;

    private final DcMotor lf, rf, lb, rb;
    private final PoseRouter router;
    private final HeadingSource headingSource;

    private double deadReckonHeading = 0;   // CCW+, used only when no heading source
    private double lastTurnCw = 0;          // last commanded turn, integrated only while it was active
    private final ElapsedTime lastDrive = new ElapsedTime();

    /** Camera-relative only: no localization, no heading sensor (scan() uses
     *  the dead-reckon estimate), no full sensor. */
    public MecanumWrangler(BallTracker tracker, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull) {
        this(tracker, lf, rf, lb, rb, intakeOrNull, null, null, null);
    }

    public MecanumWrangler(Limelight3A limelight, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull) {
        this(new BallTracker(limelight), lf, rf, lb, rb, intakeOrNull, null, null, null);
    }

    public MecanumWrangler(BallTracker tracker, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull, PoseRouter routerOrNull) {
        this(tracker, lf, rf, lb, rb, intakeOrNull, routerOrNull, null, null);
    }

    public MecanumWrangler(Limelight3A limelight, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull, PoseRouter routerOrNull) {
        this(new BallTracker(limelight), lf, rf, lb, rb, intakeOrNull, routerOrNull, null, null);
    }

    public MecanumWrangler(Limelight3A limelight, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull, PoseRouter routerOrNull, HeadingSource headingOrNull,
                           FullSensor fullOrNull) {
        this(new BallTracker(limelight), lf, rf, lb, rb, intakeOrNull, routerOrNull, headingOrNull, fullOrNull);
    }

    public MecanumWrangler(BallTracker tracker, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb,
                           DcMotor intakeOrNull, PoseRouter routerOrNull, HeadingSource headingOrNull,
                           FullSensor fullOrNull) {
        super(tracker, intakeOrNull, fullOrNull);
        this.lf = lf; this.rf = rf; this.lb = lb; this.rb = rb;
        this.router = routerOrNull;
        this.headingSource = headingOrNull;
        lastDrive.reset();
    }

    /** Standard symmetric mecanum directions (LF F, LB F, RF R, RB R). Call once
     *  in your OpMode init unless a flipped gearbox changes the wiring. */
    public static void configureMecanumDirections(DcMotor lf, DcMotor rf,
                                                   DcMotor lb, DcMotor rb) {
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* ---------------- localization (optional) ---------------- */

    @Override protected boolean hasPose() { return router != null; }

    @Override protected RobotPose robotPose() {
        return router == null ? null : router.getPose();
    }

    @Override protected boolean goToPose(RobotPose p) {
        if (router == null) return false;
        router.startGoTo(p, GO_TO_POWER);
        return true;
    }

    @Override protected boolean arrivedAtPose() {
        return router == null || !router.isBusy();
    }

    @Override protected void stopPoseMove() {
        if (router != null) router.stop();
    }

    /* ---------------- heading + rotation ---------------- */

    @Override protected double headingRad() {
        if (headingSource != null) return headingSource.headingRad();
        if (router != null) {
            RobotPose p = router.getPose();
            if (p != null) return p.headingRad;
        }
        return wrap(deadReckonHeading);
    }

    @Override protected double cwDeltaSince(double startHeadingRad) {
        return -wrap(headingRad() - startHeadingRad);
    }

    /* ---------------- motion ---------------- */

    /** While go() owns the loop, keep a PoseRouter's periodic work (Pinpoint
     *  reads, go-to PID) ticking. When update()/start() are pumped by your own
     *  loop, call loopHook() yourself each iteration. */
    @Override protected void loopHook() {
        if (router != null) router.update();
    }

    /** Mecanum mixdown, turn + = CLOCKWISE, strafe + = RIGHT (matches
     *  BallChaseController's convention). Also integrates the dead-reckoning
     *  heading so scan() estimates the sweep even with no heading sensor. The
     *  PREVIOUS command is integrated over the time elapsed since it was issued
     *  (so an idle gap doesn't credit a bogus turn), and dt is clamped so a gap
     *  can't fabricate rotation either. */
    @Override protected void driveRobot(double fwd, double strafe, double turnCw) {
        double dt = Math.min(lastDrive.seconds(), 0.1);
        lastDrive.reset();
        if (headingSource == null && router == null) {
            deadReckonHeading += -lastTurnCw * TURN_RATE_RAD_PER_POWER_SEC * dt;
        }
        lastTurnCw = turnCw;

        double pLf = fwd + strafe + turnCw;
        double pRf = fwd - strafe - turnCw;
        double pLb = fwd - strafe + turnCw;
        double pRb = fwd + strafe - turnCw;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(pLf), Math.abs(pRf)),
                         Math.max(Math.abs(pLb), Math.abs(pRb))));

        lf.setPower(pLf / max);
        rf.setPower(pRf / max);
        lb.setPower(pLb / max);
        rb.setPower(pRb / max);
    }
}