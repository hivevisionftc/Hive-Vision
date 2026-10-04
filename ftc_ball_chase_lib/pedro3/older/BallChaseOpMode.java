/*
 * Greedy Limelight 3A ball chase - camera-relative, no odometry.
 *
 * Hold gamepad1 RIGHT BUMPER to run the auto chase. Release it and you have normal
 * manual driving (left stick = drive/strafe, right stick X = turn, left bumper = intake).
 * Releasing the bumper is your instant abort.
 *
 * While the bumper is held, every loop it:
 *   1. Grabs the latest neural-detector result (skips stale ones)
 *   2. Picks the NEAREST ball of TEAM_BALL_CLASS (lowest ty = closest on the floor)
 *   3. AIMS with a P controller on tx, and only drives forward once roughly aimed
 *   4. Estimates floor distance from ty:  dist = (CAM_H - BALL_H) / tan(CAM_PITCH - ty)
 *   5. Stops at STOP_DIST, dwells so the intake can grab, then searches again
 *   6. If the ball vanishes at close range (it goes under/behind the intake), it coasts
 *      straight forward for COAST_MS instead of spinning away, then does the pickup dwell
 *
 * Conventions used everywhere below:  fwd + = forward,  strafe + = RIGHT,  turn + = CLOCKWISE.
 * Limelight tx is + when the target is to the RIGHT, so turn = +tx * gain.
 *
 * Class order on the 3A model: 0 = yellow_pollen, 1 = red_nectar, 2 = blue_nectar.
 *
 * BEFORE FIRST RUN: put the robot on blocks and check that (a) all wheels spin forward for
 * +fwd and (b) a ball on your right makes the robot turn right. Fix motor directions if not.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Ball Chase (3A Greedy)", group = "Hive Vision")
public class BallChaseOpMode extends LinearOpMode {

    /* ---- limelight / geometry ---- */
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int    PIPELINE_INDEX = 0;      // your neural detector pipeline
    private static final double CAM_PITCH_DEG  = 25.0;   // camera tilt BELOW horizontal
    private static final double CAM_H          = 12.0;   // camera lens height above floor, in
    private static final double BALL_H         = 3.0;    // ball center height above floor, in

    /* ---- detection gating ---- */
    private static final int    TEAM_BALL_CLASS  = 0;    // 0=yellow, 1=red, 2=blue
    private static final double MIN_CONF         = 0.35; // CHECK telemetry: is confidence 0-1 or 0-100 on your firmware?
    private static final long   MAX_STALENESS_US = 120_000; // ignore results older than 120 ms.
                                                           // Limelight reports staleness in MICROSECONDS.

    /* ---- approach ---- */
    private static final double STOP_DIST    = 16.0;     // camera-to-ball floor distance at pickup, in
    private static final double AIM_TOL_DEG  = 5.0;      // "aimed enough" to pick up
    private static final double DRIVE_MIN_TX = 15.0;    // beyond this many degrees off, turn in place only
    private static final double DRIVE_KP     = 0.03;     // forward power per inch of distance error
    private static final double MIN_FWD      = 0.15;     // overcome static friction
    private static final double MAX_FWD      = 0.7;

    /* ---- turning ---- */
    private static final double TURN_KP      = 0.025;    // power per degree of tx
    private static final double MIN_TURN     = 0.08;     // friction floor when outside the tolerance
    private static final double MAX_TURN     = 0.6;
    private static final double SEARCH_TURN  = 0.30;     // clockwise scan when nothing is visible

    /* ---- close-range coast + pickup ---- */
    private static final double COAST_MAX_DIST = 26.0;   // only coast if last seen within this, in
    private static final double COAST_POWER    = 0.25;
    private static final long   COAST_MS       = 450;
    private static final long   PICKUP_DWELL_MS = 600;
    private static final double INTAKE_POWER   = 1.0;

    /* ---- hardware names ---- */
    private static final String MOTOR_LF = "leftFront";
    private static final String MOTOR_RF = "rightFront";
    private static final String MOTOR_LB = "leftBack";
    private static final String MOTOR_RB = "rightBack";
    private static final String INTAKE   = "intake";     // optional; skipped if not in the config

    private enum State { SEARCHING, CHASING, COASTING, PICKUP }

    private Limelight3A limelight;
    private DcMotor lf, rf, lb, rb;
    private DcMotor intake;                              // may be null
    private State state = State.SEARCHING;

    private final ElapsedTime lastSeen = new ElapsedTime();
    private final ElapsedTime pickupTimer = new ElapsedTime();
    private double lastSeenDist = Double.MAX_VALUE;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, MOTOR_LF);
        rf = hardwareMap.get(DcMotor.class, MOTOR_RF);
        lb = hardwareMap.get(DcMotor.class, MOTOR_LB);
        rb = hardwareMap.get(DcMotor.class, MOTOR_RB);
        intake = hardwareMap.tryGet(DcMotor.class, INTAKE);

        // Positive power = forward on every wheel. Flip these if your wheels disagree.
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{lf, rf, lb, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Ready. Hold RIGHT BUMPER to auto-chase; release to drive manually.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                autoChase();
            } else {
                state = State.SEARCHING;
                manualDrive();
            }
            telemetry.addData("state", state);
            telemetry.update();
        }

        limelight.stop();
        drive(0, 0, 0);
        setIntake(0);
    }

    /* ------------------------------------------------------------------ */

    private void autoChase() {
        // Pickup dwell: hold still, run the intake, then go look for the next ball.
        if (state == State.PICKUP) {
            drive(0, 0, 0);
            setIntake(INTAKE_POWER);
            telemetry.addData("drive", "pickup - grabbing");
            if (pickupTimer.milliseconds() >= PICKUP_DWELL_MS) {
                lastSeenDist = Double.MAX_VALUE;
                state = State.SEARCHING;
            }
            return;
        }

        LLResultTypes.DetectorResult target = findTarget();

        if (target != null) {
            double tx = target.getTargetXDegrees();
            double ty = target.getTargetYDegrees();
            double dist = groundRange(ty);

            lastSeen.reset();
            lastSeenDist = dist;

            telemetry.addData("ball", "tx=%+.1f  ty=%+.1f  dist=%.1f in  conf=%.2f",
                    tx, ty, dist, target.getConfidence());

            boolean aimed = Math.abs(tx) < AIM_TOL_DEG;

            if (dist <= STOP_DIST && aimed) {
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
                return;
            }

            // Turn: P control, clockwise-positive, with a small floor outside the tolerance.
            double turn = Range.clip(tx * TURN_KP, -MAX_TURN, MAX_TURN);
            if (!aimed && Math.abs(turn) < MIN_TURN) {
                turn = Math.copySign(MIN_TURN, tx);
            }

            // Forward: only once roughly pointed at the ball, and only until STOP_DIST.
            double fwd = 0.0;
            if (dist > STOP_DIST && Math.abs(tx) < DRIVE_MIN_TX) {
                fwd = Range.clip((dist - STOP_DIST) * DRIVE_KP, MIN_FWD, MAX_FWD);
            }

            state = State.CHASING;
            setIntake(INTAKE_POWER);
            drive(fwd, 0, turn);
            telemetry.addData("drive", "fwd=%.2f turn=%.2f", fwd, turn);
            return;
        }

        // ---- no target this frame ----
        boolean wasClose = (state == State.CHASING || state == State.COASTING)
                && lastSeenDist <= COAST_MAX_DIST;

        if (wasClose) {
            if (lastSeen.milliseconds() < COAST_MS) {
                // Ball dropped out of view right at the intake: keep going straight.
                state = State.COASTING;
                setIntake(INTAKE_POWER);
                drive(COAST_POWER, 0, 0);
                telemetry.addData("ball", "lost close - coasting");
            } else {
                state = State.PICKUP;
                pickupTimer.reset();
                drive(0, 0, 0);
            }
            return;
        }

        state = State.SEARCHING;
        setIntake(0);
        drive(0, 0, SEARCH_TURN);
        telemetry.addData("ball", "no target - scanning");
    }

    /** Nearest confident ball of our class, or null. Lowest ty = closest to the robot on the floor. */
    private LLResultTypes.DetectorResult findTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || result.getStaleness() > MAX_STALENESS_US) return null;

        List<LLResultTypes.DetectorResult> dets = result.getDetectorResults();
        if (dets == null || dets.isEmpty()) return null;

        LLResultTypes.DetectorResult best = null;
        for (LLResultTypes.DetectorResult d : dets) {
            if (d.getClassId() != TEAM_BALL_CLASS) continue;
            if (d.getConfidence() < MIN_CONF) continue;
            if (best == null || d.getTargetYDegrees() < best.getTargetYDegrees()) {
                best = d;
            }
        }
        return best;
    }

    /** Floor distance camera -> ball from the vertical angle. Big number if at/above the horizon. */
    private double groundRange(double tyDeg) {
        double depression = CAM_PITCH_DEG - tyDeg;      // angle below horizontal to the ball
        if (depression < 1.0) return 999.0;
        return (CAM_H - BALL_H) / Math.tan(Math.toRadians(depression));
    }

    /* ------------------------------------------------------------------ */

    private void manualDrive() {
        double fwd    = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;
        drive(fwd, strafe, turn);
        setIntake(gamepad1.left_bumper ? INTAKE_POWER : 0);
    }

    /** Mecanum: fwd + = forward, strafe + = right, turn + = clockwise. Powers are normalized. */
    private void drive(double fwd, double strafe, double turn) {
        double pLf = fwd + strafe + turn;
        double pRf = fwd - strafe - turn;
        double pLb = fwd - strafe + turn;
        double pRb = fwd + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(pLf), Math.abs(pRf)),
                         Math.max(Math.abs(pLb), Math.abs(pRb))));

        lf.setPower(pLf / max);
        rf.setPower(pRf / max);
        lb.setPower(pLb / max);
        rb.setPower(pRb / max);
    }

    private void setIntake(double power) {
        if (intake != null) intake.setPower(power);
    }
}