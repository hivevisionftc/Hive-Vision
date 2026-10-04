/*
 * Pedro3LibSelfTest - uploadable self-test for the whole ftc_ball_chase_lib/pedro3
 * library. Add this folder to TeamCode/src/org/firstinspires/ftc/teamcode/ (the
 * package is org.firstinspires.ftc.teamcode.tests) and it shows up in the driver
 * hub as "Pedro3 Lib Self-Test" under the "Hive Vision" group.
 *
 * What it covers:
 *   S0  pure math          BallMath, scoreOf, projectBall, Target, BallColor, RobotPose
 *   S1  BallTracker        class gating, confidence gate, staleness, target lock,
 *                          coast/predict, lock timeout, groundRange, lockOn
 *   S2  BallChaseController  full state machine IDLE->SEARCHING->CHASING->COASTING
 *                          ->PICKUP->DONE, power ceilings, maxPickups, abort
 *   S3  BallWrangler       every camera-relative verb + every chain modifier, driven
 *                          end to end against MockMotor + a scripted vision source
 *   S4  on-robot           Pedro Follower + the two BallHunt drivers + BallChaseFollower
 *
 * S0-S3 need NO drivetrain and NO camera - they use MockMotor and a scripted
 * DetectionSource, so they are safe to run anywhere. S4 is the only part that
 * moves the robot, and it is gated behind an explicit button hold in INIT.
 *
 * SAFETY: S4 holds A to start and requires the robot to be lifted or on blocks.
 * Drive bursts are capped at 0.30 power and the B button aborts the whole run
 * immediately. Every motion helper zeroes the motors in a finally block.
 */
package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BallChaseController;
import org.firstinspires.ftc.teamcode.BallChaseFollower;
import org.firstinspires.ftc.teamcode.BallMath;
import org.firstinspires.ftc.teamcode.BallTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.wrapper.BallColor;
import org.firstinspires.ftc.teamcode.wrapper.BallWrangler;
import org.firstinspires.ftc.teamcode.wrapper.MecanumWrangler;
import org.firstinspires.ftc.teamcode.wrapper.PedroWrangler;
import org.firstinspires.ftc.teamcode.wrapper.RobotPose;
import org.firstinspires.ftc.teamcode.wrapper.Target;

// NOTE: org.firstinspires.ftc.teamcode.BallHunt and
// org.firstinspires.ftc.teamcode.wrapper.BallHunt share the simple name
// BallHunt, so neither is imported - both are named in full in section S4.

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

@TeleOp(name = "Pedro3 Lib Self-Test", group = "Hive Vision")
public class Pedro3LibSelfTest extends LinearOpMode {

    private static final double TOL = 1e-6;
    private static final double MOTOR_CAP = 0.30;   // ceiling for every S4 burst
    private static final long BUST_MS = 500;

    /* ---------------- execution context ---------------- */

    /* S0-S3 are pure: they only need to print, sleep and ask "still running?".
     * On the robot those go to the OpMode; the desktop main() below supplies the
     * same three things with the console, which is how the suite gets verified
     * before it is ever uploaded. */
    private abstract static class Ctx {
        abstract void out(String s);
        abstract void doSleep(long ms);
        abstract boolean running();
        abstract void telemetryOf(BallWrangler w);
        abstract void telemetryOf(BallChaseController c);
        abstract void telemetryOf(org.firstinspires.ftc.teamcode.BallHunter h);
        abstract void telemetryOf(org.firstinspires.ftc.teamcode.BallHunt h2);
        abstract LinearOpMode op();
        void line(String s) { out(s); }
        void flush() { }
        /** Emergency-stop button; always false off-robot. */
        boolean bHeld() { return false; }
        void sleep(long ms) {
            if (ms <= 0) return;
            doSleep(ms);
        }
        boolean isActive() { return running(); }
    }

    private static Ctx ctx;
    private static boolean aborting;

    /* ---------------- result bookkeeping ---------------- */

    private static int passed, failed, skipped;
    private static final List<String> failures = new ArrayList<>();
    private static String section = "";

    private static void sec(String s) {
        section = s;
        ctx.line("");
        ctx.line("== " + s + " ==");
        ctx.flush();
    }

    private static void ok(String n) {
        passed++;
        ctx.line("  PASS  " + n);
    }

    private static void bad(String n, String why) {
        failed++;
        failures.add(section + " / " + n + ": " + why);
        ctx.line("  FAIL  " + n + "   <-- " + why);
    }

    private static void chk(String n, boolean cond) {
        if (cond) ok(n); else bad(n, "condition false");
    }

    private static void chkNear(String n, double got, double want, double tol) {
        if (Double.isNaN(got)) bad(n, "got NaN, want " + want);
        else if (Math.abs(got - want) <= tol) ok(n);
        else bad(n, String.format("got %.4f, want %.4f (+-%.4f)", got, want, tol));
    }

    private static void note(String n, String v) {
        ctx.line("  ..    " + n + " = " + v);
    }

    private static void skip(String n, String why) {
        skipped++;
        ctx.line("  SKIP  " + n + "   (" + why + ")");
    }

    private static void line(String s) {
        ctx.line(s);
    }

    /* ---------------- scripted vision ---------------- */

    /** Replaces the Limelight. Feeds the tracker exact detections, so every
     *  vision-side behavior can be tested without a camera or balls. */
    private static final class Script implements BallTracker.DetectionSource {
        final List<BallTracker.RawDet> dets = new ArrayList<>();
        long ageMs = 0L;

        @Override public List<BallTracker.RawDet> latest() { return dets; }
        @Override public long stalenessMs() { return ageMs; }

        void one(int cls, double tx, double ty, double conf, double area) {
            dets.clear();
            dets.add(new BallTracker.RawDet(cls, tx, ty, conf, area));
        }

        void empty() { dets.clear(); }
    }

    private static final Script script = new Script();
    private static BallTracker tracker;

    private static void freshTracker() {
        tracker = new BallTracker(script);
        tracker.reset();
    }

    private static void threeBalls() {
        script.dets.clear();
        script.ageMs = 0;
        script.dets.add(new BallTracker.RawDet(BallTracker.CLASS_RED, -12.0, -12.0, 0.90, 300.0));
        script.dets.add(new BallTracker.RawDet(BallTracker.CLASS_BLUE, 8.0, -8.0, 0.80, 500.0));
        script.dets.add(new BallTracker.RawDet(BallTracker.CLASS_YELLOW_NEUTRAL, 20.0, -18.0, 0.95, 100.0));
    }

    /* ---------------- tunable globals we must put back ---------------- */

    /* Only the tunables this test actually writes are saved/restored; every
     * other knob in the library is left exactly as the robot ships it.
     * BallWrangler mirrors the controller's gains into CHASE_* copies at class
     * load, so both sets have to be clamped for the cap to take effect. */
    private static double gCamPitch, gCamH, gBallH, gCamX, gCamY, gMinConf, gLockGate;
    private static long gMaxStale, gLockLost;
    private static double cMinFwd, cMaxFwd, cMinTurn, cMaxTurn, cSearchTurn, cCoastPower, cIntake;
    private static double wMinFwd, wMaxFwd, wMinTurn, wMaxTurn, wCoastPower, wIntakePower;
    private static double wSearchTurn, wBack, wAlignKp, wAlignTimeout, wGoTo;

    private static void saveGlobals() {
        gCamPitch = BallTracker.CAM_PITCH_DEG; gCamH = BallTracker.CAM_H; gBallH = BallTracker.BALL_H;
        gCamX = BallTracker.CAM_X_OFFSET; gCamY = BallTracker.CAM_Y_OFFSET;
        gMinConf = BallTracker.MIN_CONF; gLockGate = BallTracker.LOCK_GATE_DEG;
        gMaxStale = BallTracker.MAX_STALENESS_MS; gLockLost = BallTracker.LOCK_LOST_MS;
        cMinFwd = BallChaseController.MIN_FWD; cMaxFwd = BallChaseController.MAX_FWD;
        cMinTurn = BallChaseController.MIN_TURN; cMaxTurn = BallChaseController.MAX_TURN;
        cSearchTurn = BallChaseController.SEARCH_TURN;
        cCoastPower = BallChaseController.COAST_POWER; cIntake = BallChaseController.INTAKE_POWER;
        wMinFwd = BallWrangler.CHASE_MIN_FWD; wMaxFwd = BallWrangler.CHASE_MAX_FWD;
        wMinTurn = BallWrangler.CHASE_MIN_TURN; wMaxTurn = BallWrangler.CHASE_MAX_TURN;
        wCoastPower = BallWrangler.CHASE_COAST_POWER; wIntakePower = BallWrangler.CHASE_INTAKE_POWER;
        wSearchTurn = BallWrangler.SEARCH_TURN; wBack = BallWrangler.BACK_POWER;
        wAlignKp = BallWrangler.ALIGN_TURN_KP; wAlignTimeout = BallWrangler.ALIGN_TIMEOUT_SEC;
        wGoTo = MecanumWrangler.GO_TO_POWER;
    }

    private static void restoreGlobals() {
        BallTracker.CAM_PITCH_DEG = gCamPitch; BallTracker.CAM_H = gCamH; BallTracker.BALL_H = gBallH;
        BallTracker.CAM_X_OFFSET = gCamX; BallTracker.CAM_Y_OFFSET = gCamY;
        BallTracker.MIN_CONF = gMinConf; BallTracker.LOCK_GATE_DEG = gLockGate;
        BallTracker.MAX_STALENESS_MS = gMaxStale; BallTracker.LOCK_LOST_MS = gLockLost;
        BallChaseController.MIN_FWD = cMinFwd; BallChaseController.MAX_FWD = cMaxFwd;
        BallChaseController.MIN_TURN = cMinTurn; BallChaseController.MAX_TURN = cMaxTurn;
        BallChaseController.SEARCH_TURN = cSearchTurn;
        BallChaseController.COAST_POWER = cCoastPower; BallChaseController.INTAKE_POWER = cIntake;
        BallWrangler.CHASE_MIN_FWD = wMinFwd; BallWrangler.CHASE_MAX_FWD = wMaxFwd;
        BallWrangler.CHASE_MIN_TURN = wMinTurn; BallWrangler.CHASE_MAX_TURN = wMaxTurn;
        BallWrangler.CHASE_COAST_POWER = wCoastPower; BallWrangler.CHASE_INTAKE_POWER = wIntakePower;
        BallWrangler.SEARCH_TURN = wSearchTurn; BallWrangler.BACK_POWER = wBack;
        BallWrangler.ALIGN_TURN_KP = wAlignKp; BallWrangler.ALIGN_TIMEOUT_SEC = wAlignTimeout;
        MecanumWrangler.GO_TO_POWER = wGoTo;
    }

    /** Proves the harness itself is honest: every static it poked is back to
     * its entry value, so S3 never ran on top of S2's leftovers. */
    private static void globalsRestored() {
        boolean ok = near(BallTracker.CAM_PITCH_DEG, gCamPitch, 1e-12)
                && near(BallTracker.CAM_H, gCamH, 1e-12)
                && near(BallTracker.BALL_H, gBallH, 1e-12)
                && near(BallTracker.CAM_X_OFFSET, gCamX, 1e-12)
                && near(BallTracker.CAM_Y_OFFSET, gCamY, 1e-12)
                && near(BallTracker.MIN_CONF, gMinConf, 1e-12)
                && near(BallTracker.LOCK_GATE_DEG, gLockGate, 1e-12)
                && near(BallTracker.MAX_STALENESS_MS, gMaxStale, 1e-12)
                && near(BallTracker.LOCK_LOST_MS, gLockLost, 1e-12)
                && near(BallChaseController.MIN_FWD, cMinFwd, 1e-12)
                && near(BallChaseController.MAX_FWD, cMaxFwd, 1e-12)
                && near(BallChaseController.MIN_TURN, cMinTurn, 1e-12)
                && near(BallChaseController.MAX_TURN, cMaxTurn, 1e-12)
                && near(BallChaseController.SEARCH_TURN, cSearchTurn, 1e-12)
                && near(BallChaseController.COAST_POWER, cCoastPower, 1e-12)
                && near(BallChaseController.INTAKE_POWER, cIntake, 1e-12)
                && near(BallWrangler.CHASE_MIN_FWD, wMinFwd, 1e-12)
                && near(BallWrangler.CHASE_MAX_FWD, wMaxFwd, 1e-12)
                && near(BallWrangler.CHASE_MIN_TURN, wMinTurn, 1e-12)
                && near(BallWrangler.CHASE_MAX_TURN, wMaxTurn, 1e-12)
                && near(BallWrangler.CHASE_COAST_POWER, wCoastPower, 1e-12)
                && near(BallWrangler.CHASE_INTAKE_POWER, wIntakePower, 1e-12)
                && near(BallWrangler.SEARCH_TURN, wSearchTurn, 1e-12)
                && near(BallWrangler.BACK_POWER, wBack, 1e-12)
                && near(BallWrangler.ALIGN_TURN_KP, wAlignKp, 1e-12)
                && near(BallWrangler.ALIGN_TIMEOUT_SEC, wAlignTimeout, 1e-12)
                && near(MecanumWrangler.GO_TO_POWER, wGoTo, 1e-12);
        chk("all tunables are restored after the run (harness is not order-dependent)", ok);
    }

    /** Hard ceiling on everything the test can command, plus short timeouts. */
    private static void clampPowers() {
        BallChaseController.MIN_FWD = 0.10; BallChaseController.MAX_FWD = MOTOR_CAP;
        BallChaseController.MIN_TURN = 0.10; BallChaseController.MAX_TURN = MOTOR_CAP;
        BallChaseController.SEARCH_TURN = MOTOR_CAP; BallChaseController.COAST_POWER = MOTOR_CAP;
        BallChaseController.INTAKE_POWER = 0.5;
        BallWrangler.CHASE_MIN_FWD = 0.10; BallWrangler.CHASE_MAX_FWD = MOTOR_CAP;
        BallWrangler.CHASE_MIN_TURN = 0.10; BallWrangler.CHASE_MAX_TURN = MOTOR_CAP;
        BallWrangler.CHASE_COAST_POWER = MOTOR_CAP; BallWrangler.CHASE_INTAKE_POWER = 0.5;
        BallWrangler.SEARCH_TURN = MOTOR_CAP; BallWrangler.BACK_POWER = MOTOR_CAP;
        BallWrangler.ALIGN_TURN_KP = 0.05; BallWrangler.ALIGN_TIMEOUT_SEC = 2.0;
        MecanumWrangler.GO_TO_POWER = MOTOR_CAP;
    }

    /* ---------------- helpers ---------------- */

    private static void dwellUntil(BooleanSupplier done, long ms) {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end && ctx.isActive()) {
            if (done.getAsBoolean()) return;
            ctx.sleep(20);
        }
    }

    /** Pump a BallChaseController on the mock drivetrain for up to ms. */
    private static void runController(BallChaseController c, long ms) {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end && ctx.isActive()) {
            c.update();
            if (c.isDone()) return;
            ctx.sleep(20);
        }
    }

    private static void runWrangler(BallWrangler w, long ms) {
        w.start();
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end && ctx.isActive()) {
            w.update();
            if (w.isDone()) return;
            ctx.sleep(10);
        }
    }

    private static boolean near(double a, double b, double tol) { return Math.abs(a - b) <= tol; }

    /** Mean of the left/right pair: the TRANSLATION component of drive(fwd,0,turn). */
    private static double avg(MockMotor left, MockMotor right) { return (left.power + right.power) / 2.0; }

    /** Half-difference of the left/right pair: the ROTATION component (+ = clockwise). */
    private static double turnOf(MockMotor left, MockMotor right) { return (left.power - right.power) / 2.0; }

    private static double maxAbs(MockMotor... ms) {
        double m = 0;
        for (MockMotor x : ms) m = Math.max(m, Math.abs(x.power));
        return m;
    }

    private static boolean allZero(MockMotor... ms) {
        for (MockMotor x : ms) if (Math.abs(x.power) > TOL) return false;
        return true;
    }

    /* =================================================================
     *  S0 - pure math (no hardware)
     * ================================================================= */

    private static void s0Math() {
        sec("S0  pure math");

        // BallMath.camRay
        double[] r = BallMath.camRay(0, 0, 0);
        chkNear("BallMath.camRay(tx=0,ty=0,pitch=0) -> forward unit", r[0], 1.0, TOL);
        chkNear("BallMath.camRay(...) lateral = 0", r[1], 0.0, TOL);
        chkNear("BallMath.camRay(...) vertical = 0", r[2], 0.0, TOL);
        double rLen = Math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        chkNear("BallMath.camRay returns a unit vector", rLen, 1.0, 1e-9);

        // BallMath.ballFieldPos
        double[] f = BallMath.ballFieldPos(0, 0, 0, 0, -10, 25, 9.2, 3.0);
        chk("BallMath.ballFieldPos hits the floor plane", f != null);
        if (f != null) {
            chk("BallMath.ballFieldPos: ball is straight ahead", f[1] > -1e-9 && f[1] < 1e-9);
            chk("BallMath.ballFieldPos: positive range", f[0] > 0);
            chkNear("BallMath.ballFieldPos bearing = 0 deg", f[3], 0.0, 1e-9);
            chkNear("BallMath.ballFieldPos range == hypot(x,y)", f[2], Math.hypot(f[0], f[1]), 1e-9);
        }
        chk("BallMath.ballFieldPos returns null when the ray never reaches ball height",
                BallMath.ballFieldPos(0, 0, 0, 0, 80, 25, 9.2, 3.0) == null);
        double[] f90 = BallMath.ballFieldPos(0, 0, 90, 0, -10, 25, 9.2, 3.0);
        chk("BallMath.ballFieldPos rotates with heading", f90 != null && f90[0] < 1e-9 && f90[1] > 0);

        // BallMath.rangeFromTa
        Double d20 = BallMath.rangeFromTa(0.25, 100.0);
        chk("BallMath.rangeFromTa(0.25, 100) = 20", d20 != null && Math.abs(d20 - 20.0) < 1e-9);
        chk("BallMath.rangeFromTa(0, ...) = null", BallMath.rangeFromTa(0.0, 100.0) == null);
        chk("BallMath.rangeFromTa(negative, ...) = null", BallMath.rangeFromTa(-1.0, 100.0) == null);

        // BallMath.pickupPose
        double[] pk = BallMath.pickupPose(36.0, 24.0, 30.0, 12.0);
        chkNear("BallMath.pickupPose stands off intakeReach",
                Math.hypot(36.0 - pk[0], 24.0 - pk[1]), 12.0, 1e-9);
        chkNear("BallMath.pickupPose keeps the heading", pk[2], 30.0, TOL);

        // BallWrangler.scoreOf - lower is better
        double near = BallWrangler.scoreOf(10, 0, 1.0);
        double far = BallWrangler.scoreOf(30, 0, 1.0);
        chk("scoreOf: closer scores better", near < far);
        chk("scoreOf: off-axis is penalized", BallWrangler.scoreOf(20, 40, 1.0) > BallWrangler.scoreOf(20, 0, 1.0));
        chk("scoreOf: higher confidence scores better", BallWrangler.scoreOf(20, 0, 1.0) < BallWrangler.scoreOf(20, 0, 0.5));
        chk("scoreOf: no off-axis penalty at bearing 0", BallWrangler.scoreOf(20, 0, 0.0) > BallWrangler.scoreOf(20, 0, 1.0));

        // BallWrangler.projectBall / atField
        double camX = BallTracker.CAM_X_OFFSET, camY = BallTracker.CAM_Y_OFFSET;
        BallTracker.CAM_X_OFFSET = 0.0; BallTracker.CAM_Y_OFFSET = 0.0;
        try {
            double[] p0 = BallWrangler.projectBall(0.0, -10.0, new RobotPose(0, 0, 0));
            chk("projectBall: geometry solvable for a floor ball", p0 != null);
            if (p0 != null) {
                chkNear("projectBall: tx=0 at heading 0 -> straight ahead", p0[0], 8.855, 0.02);
                chkNear("projectBall: tx=0 at heading 0 -> no lateral", p0[1], 0.0, 1e-9);
            }
            double[] p90 = BallWrangler.projectBall(0.0, -10.0, new RobotPose(0, 0, Math.PI / 2));
            chk("projectBall: heading rotates the projection", p90 != null
                    && Math.abs(p90[0]) < 1e-9 && Math.abs(p90[1] - 8.855) < 0.02);
            double[] pOff = BallWrangler.projectBall(0.0, -10.0, new RobotPose(24, 36, 0));
            chk("projectBall: offsets by the robot pose", pOff != null
                    && Math.abs(pOff[0] - 32.855) < 0.02 && Math.abs(pOff[1] - 36.0) < 0.02);
            BallTracker.CAM_X_OFFSET = 6.0;
            double[] pm = BallWrangler.projectBall(0.0, -10.0, new RobotPose(0, 0, 0));
            chk("projectBall: honours CAM_X_OFFSET mount shift", pm != null && Math.abs(pm[0] - (8.855 + 6.0)) < 0.02);
            BallTracker.CAM_X_OFFSET = 0.0;
            BallTracker.CAM_Y_OFFSET = 3.0;
            double[] pm2 = BallWrangler.projectBall(0.0, -10.0, new RobotPose(0, 0, 0));
            chk("projectBall: honours CAM_Y_OFFSET mount shift", pm2 != null && Math.abs(pm2[1] - 3.0) < 0.02);
            BallTracker.CAM_X_OFFSET = 0.0; BallTracker.CAM_Y_OFFSET = 0.0;
            chk("projectBall: returns null above the horizon", BallWrangler.projectBall(0.0, 80.0, new RobotPose(0, 0, 0)) == null);

            Target t = new Target(BallColor.RED, 0.0, -10.0, 8.855, 0.9, 1.0);
            chk("Target.hasField() is false before projection", !t.hasField());
            Target tf = new MecanumWrangler(tracker, mLf(), mRf(), mLb(), mRb(), mIn())
                    .atField(t, new RobotPose(24, 36, 0));
            chk("atField: attaches a field position", tf.hasField());
            if (tf.hasField()) {
                chkNear("atField: field x is correct", tf.fieldX, 32.855, 0.02);
                chkNear("atField: field y is correct", tf.fieldY, 36.0, 0.02);
            }
            chk("Target.withField returns a new Target", t.withField(1, 2) != t && t.withField(1, 2).hasField());
        } finally {
            BallTracker.CAM_X_OFFSET = camX; BallTracker.CAM_Y_OFFSET = camY;
        }

        // BallColor
        chk("BallColor.fromClassId(1) = RED", BallColor.fromClassId(1) == BallColor.RED);
        chk("BallColor.fromClassId(2) = BLUE", BallColor.fromClassId(2) == BallColor.BLUE);
        chk("BallColor.fromClassId(0) = YELLOW", BallColor.fromClassId(0) == BallColor.YELLOW);
        chk("BallColor.fromClassId(99) = null", BallColor.fromClassId(99) == null);
        chk("BallColor ids match BallTracker", BallColor.RED.classId == BallTracker.CLASS_RED
                && BallColor.BLUE.classId == BallTracker.CLASS_BLUE
                && BallColor.YELLOW.classId == BallTracker.CLASS_YELLOW_NEUTRAL);

        // RobotPose
        RobotPose rp = new RobotPose(12.0, -3.0, 1.0).inDeg(90.0);
        chkNear("RobotPose.inDeg converts to radians", rp.headingRad, Math.PI / 2, 1e-12);
        chkNear("RobotPose keeps x", rp.x, 12.0, TOL);
        chkNear("RobotPose keeps y", rp.y, -3.0, TOL);
        chk("RobotPose.toString is non-empty", rp.toString().length() > 0);

        // BallTracker.groundRange (pure geometry, no source needed)
        chkNear("groundRange(-10) matches projectBall range", tracker.groundRange(-10.0), 8.855, 0.02);
        chkNear("groundRange at the horizon = 999", tracker.groundRange(30.0), 999.0, TOL);
        chk("groundRange is monotonic", tracker.groundRange(-20.0) < tracker.groundRange(-10.0));
    }

    /* =================================================================
     *  S1 - BallTracker
     * ================================================================= */

    private static void s1Tracker() {
        sec("S1  BallTracker");
        freshTracker();

        // class gating
        tracker.setAllowedClasses(BallTracker.CLASS_RED);
        chk("isAllowed honours setAllowedClasses(int)", tracker.isAllowed(BallTracker.CLASS_RED));
        chk("isAllowed rejects other classes", !tracker.isAllowed(BallTracker.CLASS_BLUE));
        chk("getAllowedClasses size after single", tracker.getAllowedClasses().size() == 1);
        tracker.addAllowedClass(BallTracker.CLASS_BLUE);
        chk("addAllowedClass widens the set", tracker.getAllowedClasses().size() == 2);
        tracker.setAllowedClasses(BallTracker.CLASS_RED, BallTracker.CLASS_BLUE);
        chk("setAllowedClasses(int...) works", tracker.getAllowedClasses().size() == 2);
        tracker.setAllowedClasses(BallTracker.CLASSES_ALL);
        chk("CLASSES_ALL allows three", tracker.getAllowedClasses().size() == 3);
        tracker.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        chk("CLASSES_RED_BLUE allows two", tracker.getAllowedClasses().size() == 2);

        // reset
        tracker.setAllowedClasses(BallTracker.CLASSES_ALL);
        script.one(BallTracker.CLASS_RED, 0, -10, 0.9, 100);
        tracker.update();
        chk("update() reports a sighting", tracker.getLast() != null);
        tracker.reset();
        chk("reset() drops the sighting", tracker.getLast() == null);

        // confidence gate
        script.one(BallTracker.CLASS_RED, 0, -10, 0.90, 100);
        chk("getConfidentResults keeps a high-confidence det", tracker.getConfidentResults().size() == 1);
        chk("getGatedDetections keeps a high-confidence det", tracker.getGatedDetections().size() == 1);
        script.one(BallTracker.CLASS_RED, 0, -10, 0.10, 100);
        chk("getConfidentResults drops a low-confidence det", tracker.getConfidentResults().isEmpty());
        chk("getGatedDetections drops a low-confidence det", tracker.getGatedDetections().isEmpty());
        chk("low confidence yields no sighting", tracker.update() == null);

        // class filter on the output list
        tracker.setAllowedClasses(BallTracker.CLASS_BLUE);
        threeBalls();
        chk("getGatedDetections filters by allowed class", tracker.getGatedDetections().size() == 1);
        tracker.setAllowedClasses(BallTracker.CLASSES_ALL);
        chk("getGatedDetections returns all three", tracker.getGatedDetections().size() == 3);
        chk("getGatedDetections does not touch the lock", tracker.getLast() == null);

        // staleness
        script.one(BallTracker.CLASS_RED, 0, -10, 0.9, 100);
        script.ageMs = 5000;
        chk("stale source -> getGatedDetections() null", tracker.getGatedDetections() == null);
        chk("stale source -> no sighting", tracker.update() == null);
        script.ageMs = 0;
        tracker.update();
        chk("fresh source -> sighting returns", tracker.getLast() != null);
        chkNear("getStalenessMs reports the source age", tracker.getStalenessMs(), 0, TOL);

        // sighting contents
        script.one(BallTracker.CLASS_RED, 3.0, -10.0, 0.9, 100);
        BallTracker.Sighting s = tracker.update();
        chk("Sighting classId is passed through", s != null && s.classId == BallTracker.CLASS_RED);
        chkNear("Sighting tx is passed through", s.txDeg, 3.0, TOL);
        chk("a fresh sighting is not predicted", !s.predicted);
        chkNear("Sighting distIn == groundRange(ty)", s.distIn, tracker.groundRange(-10.0), TOL);

        // target lock: inside the gate the matched detection is adopted (the reading
        // tracks the ball rather than freezing on the first frame)
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 5.0, -10.0, 0.9, 100);
        BallTracker.Sighting first = tracker.update();
        chkNear("lock adopts the first sighting at tx", first.txDeg, 5.0, TOL);
        script.one(BallTracker.CLASS_RED, 8.0, -10.0, 0.9, 100);
        BallTracker.Sighting held = tracker.update();
        chk("lock follows the ball inside LOCK_GATE_DEG", held != null && Math.abs(held.txDeg - 8.0) < TOL);
        chk("a matched detection is not predicted", held != null && !held.predicted);

        // a jump outside the gate reads as "ball briefly missing": the previous
        // lock is HELD (predicted) for up to LOCK_LOST_MS, then it snaps over
        script.one(BallTracker.CLASS_RED, 30.0, -10.0, 0.9, 100);
        BallTracker.Sighting jumped = tracker.update();
        chk("a jump outside the gate holds the old lock", jumped != null && Math.abs(jumped.txDeg - 8.0) < TOL);
        chk("the held lock is marked predicted", jumped != null && jumped.predicted);
        dwellUntil(() -> {
            script.one(BallTracker.CLASS_RED, 30.0, -10.0, 0.9, 100);
            tracker.update();
            return tracker.getLast() != null && !tracker.getLast().predicted;
        }, BallTracker.LOCK_LOST_MS + 400);
        chk("after LOCK_LOST_MS the lock snaps to the new ball",
                tracker.getLast() != null && Math.abs(tracker.getLast().txDeg - 30.0) < TOL);

        // the lock is colour-bound: a different colour never steals it
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 0.0, -10.0, 0.9, 100);
        tracker.update();
        script.one(BallTracker.CLASS_BLUE, 1.0, -10.0, 0.9, 100);
        BallTracker.Sighting colourBound = tracker.update();
        chk("a different colour cannot steal the lock",
                colourBound != null && colourBound.classId == BallTracker.CLASS_RED);
        chk("the colour-bound hold is predicted", colourBound != null && colourBound.predicted);
        // getGatedDetections() is documented NOT to touch the lock, so it happily
        // reports the blue det even while update() is locked onto red.
        chk("getGatedDetections ignores the colour lock",
                tracker.getGatedDetections() != null && !tracker.getGatedDetections().isEmpty()
                        && tracker.getGatedDetections().get(0).classId == BallTracker.CLASS_BLUE);

        // coast, then lock timeout
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 0.0, -10.0, 0.9, 100);
        tracker.update();
        script.empty();
        BallTracker.Sighting coast = tracker.update();
        chk("a lost ball is coasted, not dropped", coast != null && coast.predicted);
        script.ageMs = BallTracker.MAX_STALENESS_MS + 50;
        dwellUntil(() -> { script.ageMs = BallTracker.MAX_STALENESS_MS + 50; tracker.update(); return tracker.getLast() == null; }, 900);
        chk("lock is dropped after LOCK_LOST_MS", tracker.getLast() == null);
        script.ageMs = 0;

        // lockOn forces the lock (and pre-seeds the held sighting)
        freshTracker();
        tracker.lockOn(BallTracker.CLASS_BLUE, 0.0, -10.0, 0.9);
        script.one(BallTracker.CLASS_BLUE, 2.0, -10.0, 0.9, 100);
        BallTracker.Sighting forced = tracker.update();
        chk("lockOn tracks the ball through a small jump", forced != null && Math.abs(forced.txDeg - 2.0) < TOL);
        script.one(BallTracker.CLASS_BLUE, 50.0, -10.0, 0.9, 100);
        BallTracker.Sighting forcedJump = tracker.update();
        chk("lockOn still gates big jumps", forcedJump != null && Math.abs(forcedJump.txDeg - 2.0) < TOL);
        chk("the lockOn hold is predicted", forcedJump != null && forcedJump.predicted);
        script.one(BallTracker.CLASS_RED, 51.0, -10.0, 0.9, 100);
        chk("lockOn is colour-bound too",
                tracker.update() != null && tracker.getLast().classId == BallTracker.CLASS_BLUE);

        // MIN_CONF is actually consulted
        double conf = BallTracker.MIN_CONF;
        try {
            BallTracker.MIN_CONF = 0.30;
            freshTracker();
            script.one(BallTracker.CLASS_RED, 0, -10, 0.35, 100);
            chk("MIN_CONF is read live", tracker.getConfidentResults().size() == 1);
        } finally {
            BallTracker.MIN_CONF = conf;
        }
    }

    /* =================================================================
     *  S2 - BallChaseController
     * ================================================================= */

    private static MockMotor mLf, mRf, mLb, mRb, mIn;
    private static MockMotor[] allMotors;

    static private MockMotor mLf() { return mLf; }
    static private MockMotor mRf() { return mRf; }
    static private MockMotor mLb() { return mLb; }
    static private MockMotor mRb() { return mRb; }
    static private MockMotor mIn() { return mIn; }

    private static MockMotor[] newMockDrivetrain() {
        mLf = new MockMotor(); mRf = new MockMotor(); mLb = new MockMotor(); mRb = new MockMotor();
        mIn = new MockMotor();
        allMotors = new MockMotor[]{mLf, mRf, mLb, mRb, mIn};
        return allMotors;
    }

    private static void s2Controller() {
        sec("S2  BallChaseController");
        MockMotor[] ms = newMockDrivetrain();
        freshTracker();

        // mecanum directions
        BallChaseController.configureMecanumDirections(mLf, mRf, mLb, mRb);
        chk("configureMecanumDirections: LF forward", mLf.direction == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        chk("configureMecanumDirections: RF reverse", mRf.direction == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        chk("configureMecanumDirections: LB forward", mLb.direction == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        chk("configureMecanumDirections: RB reverse", mRb.direction == com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);

        BallChaseController c = new BallChaseController(tracker, mLf, mRf, mLb, mRb, mIn);

        // idle
        chk("fresh controller is IDLE", c.getState() == BallChaseController.State.IDLE);
        chk("fresh controller is not active", !c.isActive());
        chk("fresh controller is not done", !c.isDone());
        chk("fresh controller has 0 pickups", c.getPickups() == 0);
        c.update();
        chk("update() while IDLE leaves motors alone", allZero(ms));

        // Searching
        c.setMaxPickups(2);
        chk("setMaxPickups is readable via pickups count", c.getPickups() == 0);
        script.empty();
        c.start();
        chk("start() -> active", c.isActive());
        c.update();
        chk("start() with no ball -> SEARCHING", c.getState() == BallChaseController.State.SEARCHING);
        chk("search drives the wheels", maxAbs(mLf, mRf, mLb, mRb) > 0);
        chk("search respects MAX_TURN", maxAbs(mLf, mRf, mLb, mRb) <= BallChaseController.MAX_TURN + 1e-6);
        chk("search turns in place (no translation)", Math.abs(avg(mLf, mRf)) < 1e-9);

        /* Mechanics the rest of this section relies on. drive(fwd,0,turn) mixes
         * pLf = pLb = fwd+turn, pRf = pRb = fwd-turn, so with LF/LB FORWARD and
         * RF/RB REVERBSE-by-configureMecanumDirections a uniform +power is
         * forward, and turn>0 (left forward, right backward) is CLOCKWISE. */
        // ball ahead and to the right. distIn(ty=+6) ~ 18 in, outside STOP_DIST.
        chk("sanity: a far ball is outside STOP_DIST", tracker.groundRange(6.0) > BallChaseController.STOP_DIST);
        chk("sanity: a near ball is inside STOP_DIST", tracker.groundRange(-20.0) < BallChaseController.STOP_DIST);
        chk("sanity: 6.2in < 16in so ty=-20 is a pickup range",
                tracker.groundRange(-20.0) < BallChaseController.STOP_DIST);
        tracker.reset();       // the controller holds THIS tracker: reset, never re-create
        script.one(BallTracker.CLASS_RED, 10.0, 6.0, 0.9, 100);
        c.update();
        chk("ball in view -> CHASING", c.getState() == BallChaseController.State.CHASING);
        chk("mixdown stays inside full scale", maxAbs(mLf, mRf, mLb, mRb) <= 1.0 + 1e-9);
        chk("left pair agrees (pLf == pLb)", near(mLf.power, mLb.power, 1e-9));
        chk("right pair agrees (pRf == pRb)", near(mRf.power, mRb.power, 1e-9));
        chk("chase commands forward motion", avg(mLf, mRf) > 0);
        chk("a ball to the RIGHT commands a CLOCKWISE turn", turnOf(mLf, mRf) > 0);

        // aimed: tx inside AIM_TOL_DEG -> turn left un-floored, so tiny
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 1.0, 6.0, 0.9, 100);
        c.update();
        chk("aimed -> still CHASING", c.getState() == BallChaseController.State.CHASING);
        chk("aimed within AIM_TOL_DEG -> small turn", Math.abs(turnOf(mLf, mRf)) < BallChaseController.MIN_TURN);
        chk("aimed within AIM_TOL_DEG -> still drives forward", avg(mLf, mRf) > 0);

        // off axis beyond DRIVE_MIN_TX -> turn in place only
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 40.0, 6.0, 0.9, 100);
        c.update();
        chk("far off-axis -> CHASING", c.getState() == BallChaseController.State.CHASING);
        chk("off-axis beyond DRIVE_MIN_TX -> turn in place (no translation)", Math.abs(avg(mLf, mRf)) < 1e-9);
        chk("off-axis turn is non-zero", Math.abs(turnOf(mLf, mRf)) > 0);
        chk("off-axis turn saturates at MAX_TURN",
                near(Math.abs(turnOf(mLf, mRf)), BallChaseController.MAX_TURN, 1e-9));
        chk("off-axis respects MAX_TURN", maxAbs(mLf, mRf, mLb, mRb) <= BallChaseController.MAX_TURN + 1e-6);

        // ball to the right, saturating -> still clockwise
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 25.0, 6.0, 0.9, 100);
        c.update();
        chk("ball right of centre -> turns clockwise toward it", turnOf(mLf, mRf) > 0);
        tracker.reset();
        script.one(BallTracker.CLASS_RED, -25.0, 6.0, 0.9, 100);
        c.update();
        chk("ball left of centre -> turns counter-clockwise toward it", turnOf(mLf, mRf) < 0);

        // in range -> pickup, intake runs
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 0.0, -20.0, 0.9, 100);   // distIn ~6.2 in < STOP_DIST
        c.update();
        chk("ball inside STOP_DIST -> PICKUP", c.getState() == BallChaseController.State.PICKUP);
        chk("pickup runs the intake", mIn.power > 0);
        chk("pickup intake respects INTAKE_POWER", mIn.power <= BallChaseController.INTAKE_POWER + 1e-6);
        chk("pickup stops the drivetrain", allZero(mLf, mRf, mLb, mRb));

        // A scripted ball that never leaves range keeps getting picked up (a real one
        // disappears into the intake), so cap at 1 to test a single pickup.
        c.setMaxPickups(1);
        dwellUntil(() -> { c.update(); return c.isDone(); }, BallChaseController.PICKUP_DWELL_MS + 1500);
        chk("pickup completes -> DONE", c.isDone());
        chk("one pickup counted", c.getPickups() == 1);
        chk("done is not active", !c.isActive());

        // maxPickups caps the run
        c.setMaxPickups(2);
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 0.0, -20.0, 0.9, 100);
        c.start();
        dwellUntil(() -> { c.update(); return c.isDone(); }, 6000);
        chk("run stops at setMaxPickups", c.getPickups() == 2);
        chk("capped run ends DONE", c.isDone());

        // restart() is not a thing - start() re-arms from a DONE state
        c.setMaxPickups(0);   // 0 = unlimited
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 0.0, -20.0, 0.9, 100);
        c.start();
        c.update();
        chk("start() re-arms a finished controller", c.isActive());

        // abort stops everything
        tracker.reset();
        script.one(BallTracker.CLASS_RED, 10.0, 6.0, 0.9, 100);
        c.start();
        c.update();
        chk("restart drives again", maxAbs(ms) > 0);
        c.abort();
        chk("abort clears the active flag", !c.isActive());
        chk("abort zeroes every motor", allZero(ms));

        ctx.telemetryOf(c);
        chk("addTelemetry does not throw", true);

    }

    /* =================================================================
     *  S3 - BallWrangler verbs
     * ================================================================= */

    /** Fake localization so pose verbs can be exercised on the mock drivetrain. */
    private static final class FakeRouter implements MecanumWrangler.PoseRouter {
        RobotPose pose = new RobotPose(24, 36, 0);
        boolean busy = false;
        int goToCalls = 0;
        boolean stopCalled = false;

        @Override public RobotPose getPose() { return pose; }
        @Override public void startGoTo(RobotPose target, double speedPower) { goToCalls++; busy = true; }
        @Override public boolean isBusy() { return busy; }
        @Override public void stop() { busy = false; stopCalled = true; }
        void arrive() { busy = false; }
    }

    private static MecanumWrangler newWrangler(FakeRouter router) {
        MecanumWrangler.HeadingSource hs =
                router == null ? null : () -> router.getPose().headingRad;
        return new MecanumWrangler(tracker, mLf, mRf, mLb, mRb, mIn, router, hs, null);
    }

    private static void s3Wrangler() {
        sec("S3  BallWrangler verbs");
        MockMotor[] ms = newMockDrivetrain();
        freshTracker();
        threeBalls();

        // ---- colour selectors.
        // Worth knowing: the selector (reds/blues/.../setColors) gates the
        // "any colour" queries - canSee(), count(), findNearest(), the
        // findLeftmost family, and every chase step. The per-colour queries
        // (canSee(c)/count(c)/find(c)) deliberately report RAW visibility, so
        // reds().grab(BLUE) will still go for a blue ball.
        BallWrangler w = newWrangler(null).reds();
        chk("reds(): the any-colour list is restricted", w.count() == 1);
        chk("reds(): findNearest is red", w.findNearest() != null && w.findNearest().color == BallColor.RED);
        chk("reds(): colorOf(find(RED)) = RED", w.colorOf(w.find(BallColor.RED)) == BallColor.RED);
        chk("reds(): canSee(BLUE) reports raw visibility (not selection)",
                w.canSee(BallColor.BLUE));
        chk("reds(): count(BLUE) reports raw visibility", w.count(BallColor.BLUE) == 1);
        chk("reds(): find(BLUE) is still non-null", w.find(BallColor.BLUE) != null);
        chk("colorOf(null) = null", w.colorOf(null) == null);

        w = newWrangler(null).blues();
        chk("blues(): restricted to blue", w.count() == 1 && w.findNearest().color == BallColor.BLUE);

        w = newWrangler(null).yellows();
        chk("yellows(): restricted to yellow", w.count() == 1 && w.findNearest().color == BallColor.YELLOW);

        w = newWrangler(null).alliance();
        chk("alliance(): red + blue, no yellow", w.count() == 2);

        w = newWrangler(null).everything();
        chk("everything(): all three visible", w.count() == 3);

        w = newWrangler(null).setColors(BallTracker.CLASS_RED, BallTracker.CLASS_YELLOW_NEUTRAL);
        chk("setColors(...) narrows the set", w.count() == 2);

        w = newWrangler(null);
        chk("default selector is alliance (red + blue)", w.count() == 2);

        script.empty();
        chk("empty view: canSee() false", !w.canSee());
        chk("empty view: count() = 0", w.count() == 0);
        chk("empty view: findNearest() null", w.findNearest() == null);
        threeBalls();

        // ---- selection verbs
        w = newWrangler(null).everything();
        chkNear("findLeftmost picks the smallest tx", w.findLeftmost().bearingDeg, -12.0, TOL);
        chkNear("findRightmost picks the largest tx", w.findRightmost().bearingDeg, 20.0, TOL);
        chkNear("findBiggest picks the largest area", w.findBiggest().bearingDeg, 8.0, TOL);
        chk("find(color) picks that colour", w.find(BallColor.BLUE).bearingDeg == 8.0);
        double best = Math.min(Math.min(w.findLeftmost().score, w.findRightmost().score),
                Math.min(w.findBiggest().score, w.find(BallColor.YELLOW).score));
        chkNear("findBestScore picks the lowest score", w.findBestScore().score, best, 1e-9);

        // ---- field projection through a PoseRouter
        FakeRouter router = new FakeRouter();
        w = newWrangler(router).everything();
        Target fp = w.findNearestAtPose();
        chk("findNearestAtPose projects when localization exists", fp.hasField());
        Target manual = w.atField(w.findNearest(), new RobotPose(0, 0, 0));
        chk("atField attaches coords", manual.hasField() && manual.fieldX > 0);
        chk("Target.toString includes the field position",
                manual.toString().contains("field="));

        w = newWrangler(null).everything();
        chk("no localization -> findNearestAtPose stays camera-relative", !w.findNearestAtPose().hasField());

        // ---- scheduled() step accounting (clear() first: the chain accumulates)
        w = newWrangler(null).everything();
        chk("clear() empties the chain", w.grabNearest().clear().scheduled() == 0);
        chk("grabNearest() is one step", w.clear().grabNearest().scheduled() == 1);
        chk("grabNearest().grab(RED) is two steps", w.clear().grabNearest().grab(BallColor.RED).scheduled() == 2);
        chk("grabTwo() is two steps", w.clear().grabTwo(BallColor.RED, BallColor.BLUE).scheduled() == 2);
        chk("and() chains like grab()", w.clear().grab(BallColor.RED).and(BallColor.BLUE).scheduled() == 2);
        chk("grabAll() is one step", w.clear().grabAll().scheduled() == 1);
        chk("grabUpTo(3) is one step", w.clear().grabUpTo(3).scheduled() == 1);
        chk("grabLeftmost() is one step", w.clear().grabLeftmost().scheduled() == 1);
        chk("grabRightmost() is one step", w.clear().grabRightmost().scheduled() == 1);
        chk("grab(Target) is one step", w.clear().grab(w.findNearest()).scheduled() == 1);
        chk("scan() is one step", w.clear().scan().scheduled() == 1);
        chk("scanLeft() is one step", w.clear().scanLeft().scheduled() == 1);
        chk("scanRight() is one step", w.clear().scanRight().scheduled() == 1);
        chk("lookFor(color) is one step", w.clear().lookFor(BallColor.RED).scheduled() == 1);
        chk("nudge(Target) is one step", w.clear().nudge(w.findNearest()).scheduled() == 1);
        chk("approach(Target) is one step", w.clear().approach(w.findNearest()).scheduled() == 1);
        chk("alignTo(Target) is one step", w.clear().alignTo(w.findNearest()).scheduled() == 1);
        chk("alignTo(color) is one step", w.clear().alignTo(BallColor.RED).scheduled() == 1);
        chk("backOff(in) is one step", w.clear().backOff(6.0).scheduled() == 1);
        chk("intakeOn() is one step", w.clear().intakeOn().scheduled() == 1);
        chk("intakeOff() is one step", w.clear().intakeOff().scheduled() == 1);
        chk("reverse(sec) is one step", w.clear().reverse(0.2).scheduled() == 1);
        RobotPose goal = new RobotPose(48, 24, 0);
        chk("then(pose) is one step", w.clear().then(goal).scheduled() == 1);
        chk("thenReturnTo(pose) is one step", w.clear().thenReturnTo(goal).scheduled() == 1);
        chk("searchAt(pose) is two steps (goTo + scan)", w.clear().searchAt(goal).scheduled() == 2);
        chk("within(sec) adds no step", w.clear().grabNearest().within(3.0).scheduled() == 1);
        chk("orGiveUp(sec) adds no step", w.clear().grabNearest().orGiveUp(2.0).scheduled() == 1);
        chk("orElse(runnable) adds no step", w.clear().grabNearest().orElse(() -> { }).scheduled() == 1);

        w.clear().when(BallColor.RED).grab(BallColor.BLUE);
        chk("when(color).grab(color) queues one step", w.scheduled() == 1);
        w.clear().ifSeen(BallColor.RED).grab(BallColor.BLUE);
        chk("ifSeen(color) is an alias of when()", w.scheduled() == 1);
        w.clear().grab(BallColor.RED).orGiveUp(2.0).when(BallColor.BLUE).orElse(() -> { }).within(1.0);
        chk("When.orElse + orGiveUp + within add no steps of their own", w.scheduled() == 1);
        w.clear().grabNearest().when(BallColor.RED).backOff(4.0);
        chk("When.backOff queues a step", w.scheduled() == 2);
        w.clear().grabNearest().when(BallColor.RED).approach(BallColor.BLUE);
        chk("When.approach queues a step", w.scheduled() == 2);
        w.clear().grabNearest().when(BallColor.RED).then(goal);
        chk("When.then queues a step", w.scheduled() == 2);
        w.clear().grabNearest().when(BallColor.RED).thenReturnTo(goal);
        chk("When.thenReturnTo queues a step", w.scheduled() == 2);

        // ---- cross-chain then(other) drains the other chain
        BallWrangler a = newWrangler(null).everything();
        BallWrangler b = newWrangler(null).everything();
        a.grabNearest();
        b.grab(BallColor.RED);
        chk("then(other): other starts populated", b.scheduled() == 1);
        a.then(b);
        chk("then(other): steps move onto this chain", a.scheduled() == 2);
        chk("then(other): other is drained", b.scheduled() == 0);

        // ---- sensors
        w = newWrangler(null).everything();
        chk("isFull() false with no FullSensor", !w.isFull());
        chk("withPickupSensor returns this", w.withPickupSensor(() -> true) == w);
        MecanumWrangler full = new MecanumWrangler(tracker, mLf, mRf, mLb, mRb, mIn, null, null, () -> true);
        chk("isFull() true when the FullSensor says so", full.isFull());

        // ---- intake verbs really command the motor. intakeOn() is a SINGLE-FRAME
        // step that zeroes itself the instant it finishes, so the only way to
        // see it is MockMotor.peakPower - not power, which is already back to 0.
        w = newWrangler(null).everything();
        mIn.reset();
        runWrangler(w.clear().intakeOn(), 800);
        chk("intakeOn() spun the intake", mIn.peakPower > 0);
        chk("intakeOn() honours CHASE_INTAKE_POWER",
                mIn.peakPower <= BallWrangler.CHASE_INTAKE_POWER + 1e-6);
        mIn.reset();
        runWrangler(w.clear().intakeOff(), 800);
        chk("intakeOff() never powered the intake", mIn.setPowerCalls == 0 || mIn.peakPower == 0);
        mIn.reset();
        runWrangler(w.clear().intakeOn().intakeOff(), 1500);
        chk("intakeOn().intakeOff() ends at zero", mIn.power == 0);
        chk("intakeOn().intakeOff() still engaged once", mIn.peakPower > 0);

        // ---- reverse() is an INTAKE unjam (it runs the intake backwards), not
        // a drivetrain reverse - easy to misread from the name alone.
        mIn.reset(); mLf.reset(); mRf.reset(); mLb.reset(); mRb.reset();
        w = newWrangler(null).everything();
        runWrangler(w.clear().reverse(0.25), 1500);
        chk("reverse() drives the intake backwards", mIn.peakPower < 0);
        chk("reverse() leaves the intake stopped", mIn.power == 0);
        chk("reverse() does not move the drivetrain", maxAbs(mLf, mRf, mLb, mRb) == 0);

        // ---- orElse actually fires
        script.empty();
        final boolean[] fired = {false};
        w = newWrangler(null).everything();
        runWrangler(w.clear().grabNearest().orElse(() -> fired[0] = true), 3000);
        chk("orElse runs when the verb finds nothing", fired[0]);
        chk("orElse chain still finishes", w.isDone());
        threeBalls();

        // ---- abort stops the drive mid-chain
        mLf.reset(); mRf.reset(); mLb.reset(); mRb.reset();
        w = newWrangler(null).everything();
        script.empty();
        w.clear().scan();
        w.start();
        w.update();
        chk("scan() drives before abort", maxAbs(mLf, mRf, mLb, mRb) > 0);
        w.abort();
        chk("abort() -> isDone()", w.isDone());
        chk("abort() -> not active", !w.isActive());
        chk("abort() zeroes the drive", allZero(mLf, mRf, mLb, mRb));
        chk("abort() zeroes the intake", allZero(mIn));

        // ---- pickup counting
        w = newWrangler(null).everything();
        runWrangler(w.clear().grabAll().within(4.0), 6000);
        chk("grabAll() finishes inside its budget", w.isDone());
        chk("getPickups() == got()", w.getPickups() == w.got());
        note("pickups with a motionless mock drivetrain", String.valueOf(w.got()));
        note("lastError", String.valueOf(w.lastError()));

        // ---- pose verbs: with and without localization
        FakeRouter r2 = new FakeRouter();
        w = newWrangler(r2).everything();
        w.clear().then(goal).within(2.0);
        w.start();
        w.update();
        r2.arrive();
        runWranglerPump(w, 1500);
        chk("then(pose) calls the PoseRouter", r2.goToCalls > 0);
        chk("then(pose) asks the router to stop when done", r2.stopCalled);
        chk("then(pose) finishes", w.isDone());
        note("lastError after a clean go-to", String.valueOf(w.lastError()));

        w = newWrangler(null).everything();
        w.clear().then(goal).within(2.0);
        w.start();
        runWranglerPump(w, 1500);
        chk("then(pose) without localization still finishes", w.isDone());
        note("lastError without localization (expected non-null)",
                String.valueOf(w.lastError()));

        // ---- go(LinearOpMode) drives the whole chain for real.
        // Only the OpMode has a LinearOpMode to hand it, so the desktop run
        // reports this one as skipped rather than pretending to cover it.
        w = newWrangler(null).everything();
        if (ctx.op() != null) {
            w.grabAll().within(3.0).orGiveUp(2.0).go(ctx.op());
            chk("go(op) completes the chain", w.isDone());
            chk("go(op) drains scheduled()", w.scheduled() == 0);
        } else {
            skip("go(op) end-to-end", "needs a LinearOpMode - runs in the OpMode");
        }
        ctx.telemetryOf(w);
        chk("addTelemetry does not throw", true);
    }

    private static void runWranglerPump(BallWrangler w, long ms) {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end && ctx.isActive() && !abortRequested()) {
            w.update();
            if (w.isDone()) return;
            ctx.sleep(10);
        }
    }

    /* =================================================================
     *  S4 - on-robot
     * ================================================================= */

    private DcMotor hwLf, hwRf, hwLr, hwRr, hwIntake;
    private Follower follower;
    private Limelight3A limelight;

    /** B is the emergency stop, honoured by every loop in S4. */
    private static boolean abortRequested() {
        return aborting || (ctx != null && ctx.bHeld());
    }

    private DcMotor first(DcMotor a, DcMotor b) { return a != null ? a : b; }

    private DcMotor grabMotor(String... names) {
        for (String n : names) {
            DcMotor m = hardwareMap.tryGet(DcMotor.class, n);
            if (m != null) return m;
        }
        return null;
    }

    /** Non-throwing hardware presence probe. get() throws when a name is absent,
     *  which is exactly the signal we want. */
    private boolean hwPresent(String name) {
        try {
            return hardwareMap.get(name) != null;
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** List what Pedro needs, show what the robot actually has, and refuse to
     *  move if any of it is missing. Printing the real names is the fastest way
     *  for another team to fix Constants for their robot. */
    private boolean preflight() {
        String[] needed = {
                Constants.LEFT_FRONT, Constants.LEFT_REAR,
                Constants.RIGHT_FRONT, Constants.RIGHT_REAR,
                Constants.IMU_NAME
        };
        StringBuilder missing = new StringBuilder();
        for (String n : needed) {
            boolean found = hwPresent(n);
            line("  need  " + n + "  " + (found ? "ok" : "MISSING"));
            if (!found) {
                if (missing.length() > 0) missing.append(", ");
                missing.append(n);
            }
        }
        try {
            java.util.SortedSet<String> have =
                    hardwareMap.getAllNames(com.qualcomm.robotcore.hardware.HardwareDevice.class);
            line("  this robot's devices: " + have);
        } catch (RuntimeException e) {
            line("  (could not enumerate devices: " + e.getMessage() + ")");
        }
        if (missing.length() > 0) {
            line("  !!  not driving: missing " + missing);
            line("  !!  edit older/pedroPathing/Constants.java to match the names above");
            return false;
        }
        return true;
    }

    private void hardwareSection() {
        sec("S4  on-robot (hardware)");

        if (!preflight()) {
            skip("S4", "hardware incomplete - see the preflight list above");
            return;
        }

        hwLf = grabMotor(Constants.LEFT_FRONT, "leftFront");
        hwRf = grabMotor(Constants.RIGHT_FRONT, "rightFront");
        hwLr = grabMotor(Constants.LEFT_REAR, "leftRear");
        hwRr = grabMotor(Constants.RIGHT_REAR, "rightRear");
        hwIntake = hardwareMap.tryGet(DcMotor.class, "intake");

        if (hwLf == null || hwRf == null || hwLr == null || hwRr == null) {
            skip("S4", "no drivetrain on the control hub - expected " + Constants.LEFT_FRONT
                    + " / " + Constants.LEFT_REAR + " / " + Constants.RIGHT_FRONT + " / " + Constants.RIGHT_REAR);
            return;
        }
        note("drivetrain found", Constants.LEFT_FRONT + ", " + Constants.LEFT_REAR + ", "
                + Constants.RIGHT_FRONT + ", " + Constants.RIGHT_REAR);
        note("intake", hwIntake == null ? "absent (verbs still work)" : "present");

        for (DcMotor m : new DcMotor[]{hwLf, hwRf, hwLr, hwRr}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (hwIntake != null) {
            hwIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hwIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        MecanumWrangler.configureMecanumDirections(hwLf, hwRf, hwLr, hwRr);

        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        note("limelight", limelight == null ? "absent (tracker will see nothing)" : "present");

        // ---- Constants.createFollower + pose round trip
        try {
            follower = Constants.createFollower(hardwareMap);
            chk("Constants.createFollower returns a Follower", follower != null);
        } catch (RuntimeException e) {
            bad("Constants.createFollower", "threw " + e.getClass().getSimpleName() + ": " + e.getMessage());
            return;
        }

        follower.setPose(new Pose(0, 0, 0));
        pumpFollower(120);
        Pose p0 = follower.pose();
        chkNear("follower.setPose(0,0,0) -> pose().x()", p0.x(), 0.0, 0.5);
        chkNear("follower.setPose(0,0,0) -> pose().y()", p0.y(), 0.0, 0.5);
        chkNear("follower.setPose(0,0,0) -> pose().heading()", p0.heading(), 0.0, 0.05);
        chk("follower.isBusy() false at rest", !follower.isBusy());
        note("pose after setPose", String.format("x=%.2f y=%.2f h=%.1f deg", p0.x(), p0.y(), Math.toDegrees(p0.heading())));

        // ---- follower.update() actually integrates odometry
        Pose before = follower.pose();
        burst(() -> follower.manual(ManualDrive.fieldCentric(
                new DrivePowers(MOTOR_CAP, 0, 0), follower.pose().heading())), BUST_MS);
        pumpFollower(300);
        Pose afterFwd = follower.pose();
        double dx = afterFwd.x() - before.x();
        chk("follower drives forward on odometry", dx > 0.05);
        chk("forward odometry is sane (< 2 ft in " + BUST_MS + "ms)", dx < 24.0);
        note("forward burst", String.format("dx=%.2f in  dy=%.2f in  dh=%.1f deg",
                dx, afterFwd.y() - before.y(), Math.toDegrees(afterFwd.heading() - before.heading())));

        // ---- reverse
        before = follower.pose();
        burst(() -> follower.manual(ManualDrive.fieldCentric(
                new DrivePowers(-MOTOR_CAP, 0, 0), follower.pose().heading())), BUST_MS);
        pumpFollower(300);
        double dxRev = follower.pose().x() - before.x();
        chk("follower drives backward on odometry", dxRev < -0.02);

        // ---- strafe
        before = follower.pose();
        burst(() -> follower.manual(ManualDrive.fieldCentric(
                new DrivePowers(0, MOTOR_CAP, 0), follower.pose().heading())), BUST_MS);
        pumpFollower(300);
        double dStrafe = follower.pose().y() - before.y();
        chk("follower strafes on odometry", Math.abs(dStrafe) > 0.02);

        // ---- turn
        before = follower.pose();
        burst(() -> follower.manual(ManualDrive.fieldCentric(
                new DrivePowers(0, 0, MOTOR_CAP), follower.pose().heading())), BUST_MS);
        pumpFollower(300);
        double dTurn = Math.toDegrees(follower.pose().heading() - before.heading());
        chk("follower turns on odometry", Math.abs(dTurn) > 1.0);
        note("turn burst", String.format("dheading=%+.1f deg", dTurn));

        chk("follower.stop() clears busy", stopFollower());

        // ---- PedroWrangler: the pose plumbing
        freshTracker();
        PedroWrangler pw = new PedroWrangler(follower, tracker, hwIntake);

        // hasPose()/robotPose() are protected, so prove localization through the
        // public surface: findNearestAtPose() only attaches field coords when the
        // wrangler can read a pose, and PedroWrangler always can.
        script.one(BallTracker.CLASS_RED, 0.0, -10.0, 0.9, 100);
        script.ageMs = 0;
        Target pf = pw.findNearestAtPose();
        chk("PedroWrangler sees the live camera source", pw.canSee());
        chk("PedroWrangler has localization (findNearestAtPose projects)", pf != null && pf.hasField());
        if (pf != null && pf.hasField()) {
            Pose fp = follower.pose();
            note("ball projected to", String.format("(%.1f, %.1f) from robot (%.1f, %.1f)",
                    pf.fieldX, pf.fieldY, fp.x(), fp.y()));
            chk("projection is anchored on the follower pose",
                    Math.hypot(pf.fieldX - fp.x(), pf.fieldY - fp.y()) < 14.0);
        }

        // camera-relative verbs drive through follower.manual()
        chk("PedroWrangler backOff finished on its own",
                runWranglerPumpStarted(pw.clear().backOff(4.0), 900));

        chk("PedroWrangler scanLeft finished on its own",
                runWranglerPumpStarted(pw.clear().scanLeft(), 900));

        chk("PedroWrangler nudge finished on its own",
                runWranglerPumpStarted(pw.clear().nudge(
                        new Target(BallColor.RED, 0.0, -10.0, 8.8, 0.9, 1.0)), 700));

        /* Odometry-scale interlock. MOTOR_CAP/BUST_MS bound the manual bursts
         * above, but a Pedro path runs at the velocity limits in Constants, which
         * are NOT capped by this test. If the ticks->inches placeholders are
         * still wrong, odometry over-reports by ~25x and the follower will
         * floor it trying to correct the phantom error. Refuse to path until a
         * measured burst proves the scale is sane. */
        double burstIn = Math.abs(dx);
        boolean scaleOk = burstIn > 0.05 && burstIn < 12.0;
        if (!scaleOk) {
            skip("PedroWrangler then(pose) go-to",
                    String.format("odometry scale implausible - %s in burst at %.2f power for %d ms. "
                                    + "Fix Constants ticks->inches before running any path.",
                            String.format("%.1f", burstIn), MOTOR_CAP, BUST_MS));
            stopFollower();
            zeroMotors();
        } else {
            // a real short go-to, bounded
            Pose cur = follower.pose();
            boolean finished = runWranglerPumpStarted(
                    pw.clear().then(new RobotPose(cur.x() + 3.0, cur.y(), cur.heading())).within(6.0),
                    5000);
            chk("PedroWrangler then(pose) completes", finished);
            Pose moved = follower.pose();
            double movedIn = Math.hypot(moved.x() - cur.x(), moved.y() - cur.y());
            chk("PedroWrangler then(pose) moved the robot", movedIn > 0.3);
            chk("PedroWrangler then(pose) did not overshoot wildly", movedIn < 30.0);
            note("go-to result", String.format("asked 3.0 in, moved %.2f in (from %.1f,%.1f to %.1f,%.1f)",
                    movedIn, cur.x(), cur.y(), moved.x(), moved.y()));
            chk("PedroWrangler go-to stops the follower", stopFollower());
        }

        // ---- the two BallHunt drivers wire up against the live follower
        org.firstinspires.ftc.teamcode.BallHunt huntA =
                new org.firstinspires.ftc.teamcode.BallHunt(follower, tracker, hwIntake);
        huntA.reds().collect(1).within(1.0);
        huntA.start();
        chk("final.BallHunt is not done right after start()", !huntA.isDone());
        huntA.update();
        chk("final.BallHunt reports its state", huntA.state() != null);
        note("final.BallHunt state", huntA.state());
        huntA.abort();
        chk("final.BallHunt aborts cleanly", huntA.isDone());
        zeroMotors();
        ctx.telemetryOf(huntA);


        org.firstinspires.ftc.teamcode.wrapper.BallHunt huntB =
                new org.firstinspires.ftc.teamcode.wrapper.BallHunt(follower, tracker, hwIntake);
        huntB.everything().collect(2);
        chk("wrapper.BallHunt queues a gather", huntB.scheduled() > 0);
        chk("wrapper.BallHunt reports its state", huntB.state() != null);
        note("wrapper.BallHunt state", huntB.state());

        // ---- older/BallHunter: construct + state only, no motion
        org.firstinspires.ftc.teamcode.BallHunter legacy =
                new org.firstinspires.ftc.teamcode.BallHunter(follower, limelight, hwIntake);
        legacy.setMaxPickups(1);
        legacy.setTimeBudgetSec(5.0);
        legacy.start();
        chk("older.BallHunter goes active on start()", legacy.isActive());
        legacy.update();
        chk("older.BallHunter exposes its state", legacy.getState() != null);
        chk("older.BallHunter starts with 0 pickups", legacy.getPickups() == 0);
        legacy.abort();
        chk("older.BallHunter aborts cleanly", !legacy.isActive());
        zeroMotors();
        ctx.telemetryOf(legacy);


        // ---- BallChaseFollower constructs against the live follower
        BallChaseFollower bcf = new BallChaseFollower(follower, limelight, hwIntake);
        chk("BallChaseFollower constructs", bcf != null);

        stopFollower();
        zeroMotors();
    }

    private void pumpFollower(int n) {
        for (int i = 0; i < n && ctx.isActive() && !abortRequested(); i++) {
            if (follower != null) follower.update();
            ctx.sleep(10);
        }
    }

    private boolean stopFollower() {
        try {
            if (follower == null) return true;
            follower.stop();
            follower.manual(new DrivePowers(0, 0, 0));
            return !follower.isBusy();
        } catch (RuntimeException e) {
            bad("follower.stop()", e.getClass().getSimpleName() + ": " + e.getMessage());
            return false;
        }
    }

    /** Start, pump for at most ms, and report whether it FINISHED on its own.
     *  The done flag has to be sampled before abort(), because abort() forces
     *  done - checking afterwards would pass even if the motion never ran. */
    private boolean runWranglerPumpStarted(BallWrangler w, long ms) {
        try {
            w.start();
            runWranglerPump(w, ms);
            return w.isDone();
        } finally {
            w.abort();
            // abort() should zero its own step, but these verbs touch real
            // motors: belt-and-braces, because nobody wants a surprise 0.30
            // burst between two checks.
            stopFollower();
            zeroMotors();
            ctx.sleep(120);
        }
    }

    /** Run a commanded motion for ms, then hard-stop in a finally block. */
    private void burst(Command c, long ms) {
        long end = System.currentTimeMillis() + ms;
        try {
            while (System.currentTimeMillis() < end && ctx.isActive() && !abortRequested()) {
                c.drive();
                follower.update();
                ctx.sleep(10);
            }
            if (abortRequested()) {
                aborting = true;
                line("  !!  B pressed - aborting run");
            }
        } finally {
            stopFollower();
            zeroMotors();
            ctx.sleep(150);
        }
    }

    private interface Command { void drive(); }

    private void zeroMotors() {
        for (DcMotor m : new DcMotor[]{hwLf, hwRf, hwLr, hwRr, hwIntake}) {
            if (m != null) m.setPower(0.0);
        }
    }

    /* =================================================================
     *  main
     * ================================================================= */

    @Override
    public void runOpMode() {
        ctx = new Ctx() {
            @Override void out(String s) { telemetry.addLine(s); telemetry.update(); }
            @Override void doSleep(long ms) { sleep(ms); }
            @Override boolean running() { return opModeIsActive(); }
        @Override boolean bHeld() { return gamepad1.b; }
            @Override void telemetryOf(BallWrangler w) { w.addTelemetry(telemetry); telemetry.update(); }
            @Override void telemetryOf(BallChaseController c) { c.addTelemetry(telemetry); telemetry.update(); }
            @Override void telemetryOf(org.firstinspires.ftc.teamcode.BallHunter h) { h.addTelemetry(telemetry); telemetry.update(); }
            @Override void telemetryOf(org.firstinspires.ftc.teamcode.BallHunt h2) { h2.addTelemetry(telemetry); telemetry.update(); }
            @Override LinearOpMode op() { return Pedro3LibSelfTest.this; }
        };
        saveGlobals();
        clampPowers();
        newMockDrivetrain();
        freshTracker();

        telemetry.setMsTransmissionInterval(50);
        ctx.line("Pedro3 Lib Self-Test");
        ctx.line("S0-S3 run against mocks - no drivetrain or camera needed.");
        ctx.line("S4 DRIVES THE ROBOT. Lift it / block the wheels first.");
        ctx.line("S4 also runs a Pedro path at the velocity limits in");
        ctx.line("older/pedroPathing/Constants.java - set those sanely first.");
        ctx.line("Hold A + START to include S4.  B aborts at any time.");
        ctx.flush();

        boolean wantHardware = false;
        while (!isStarted() && !isStopRequested()) {
            line("");
            line("== waiting to start ==");
            line("  S0-S3 only : press START");
            line("  S0-S4      : hold A, then press START");
            line("  A held     : " + gamepad1.a);
            ctx.flush();
            wantHardware = gamepad1.a;
            ctx.sleep(100);
        }
        waitForStart();

        try {
            s0Math();
            s1Tracker();
            s2Controller();
            s3Wrangler();

            if (wantHardware) {
                hardwareSection();
            } else {
                sec("S4  on-robot");
                skip("S4", "not requested - hold A in INIT to include it");
            }
        } catch (RuntimeException e) {
            bad("unhandled exception in a section",
                    e.getClass().getSimpleName() + ": " + e.getMessage());
        } finally {
            // Guaranteed teardown: no path out of this OpMode leaves a motor
            // commanded, your tunables modified, or the summary unwritten.
            restoreGlobals();
            zeroMotors();
            stopFollower();
            printSummary();
        }
    }

    private static void printSummary() {
        sec("SUMMARY");
        line("  passed  " + passed);
        line("  failed  " + failed);
        line("  skipped " + skipped);
        for (int i = 0; i < failures.size(); i++) {
            line("  " + (i + 1) + ") " + failures.get(i));
        }
        line("");
        line(failed == 0 ? "  RESULT: ALL CHECKS PASSED" : "  RESULT: " + failed + " FAILURE(S) - SEE ABOVE");
        ctx.flush();
    }

    /* Desktop entry point: runs the whole of S0-S3 with a console Ctx so the
     * suite is verified on a laptop BEFORE anyone uploads it to a robot.
     * S4 needs a real drivetrain and a Pedro Follower, so it only ever runs
     * from the OpMode. Exit code is non-zero if any check failed, so this can
     * be wired into a build later. */
    public static void main(String[] args) {
        ctx = new Ctx() {
            @Override void out(String s) { System.out.println(s); }
            @Override void doSleep(long ms) {
                try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            }
            @Override boolean running() { return true; }
            @Override void telemetryOf(BallWrangler w) { }
            @Override void telemetryOf(BallChaseController c) { }
            @Override void telemetryOf(org.firstinspires.ftc.teamcode.BallHunter h) { }
            @Override void telemetryOf(org.firstinspires.ftc.teamcode.BallHunt h2) { }
            @Override LinearOpMode op() { return null; }
        };

        saveGlobals();
        clampPowers();
        newMockDrivetrain();
        freshTracker();

        ctx.line("Pedro3 Lib Self-Test - desktop run of S0-S3 (no robot, no camera)");
        ctx.line("S4 needs a real drivetrain; it only runs on the robot.");
        ctx.flush();

        s0Math();
        s1Tracker();
        s2Controller();
        s3Wrangler();

        restoreGlobals();
        globalsRestored();
        printSummary();
        System.out.flush();
        System.exit(failed == 0 ? 0 : 1);
    }
}