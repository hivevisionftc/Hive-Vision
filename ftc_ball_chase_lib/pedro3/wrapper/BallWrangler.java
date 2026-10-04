/*
 * BallWrangler - fluent verb API for ball collecting, drivetrain-agnostic.
 * Subclasses (PedroWrangler, MecanumWrangler) supply only the physical motion;
 * everything else - perception, deciding, intake, chaining - lives here.
 *
 * An auto reads like prose:
 *
 *     BallHunt hunt = new BallHunt(follower, limelight, intake);
 *
 *     if (hunt.canSee(RED)) hunt.grab(RED).within(6).go(this);
 *     else                  hunt.scan().then(hunt.grabNearest()).go(this);
 *
 *     hunt.grabTwo(RED, BLUE).thenReturnTo(scorePose).go(this);
 *
 * Verb groups
 *   finding (look, don't drive):    canSee / count / findNearest / find /
 *                                   findLeftmost / findRightmost / findBiggest /
 *                                   findBestScore
 *   grabbing (drive + intake):      grabNearest / grab(color) / grab(target) /
 *                                   grabLeftmost / grabRightmost / grabAll /
 *                                   grabUpTo / grabTwo / and
 *   searching:                      scan / scanLeft / scanRight / searchAt /
 *                                   lookFor(color).orGiveUp(sec)
 *   moving around a ball:           approach(target) / alignTo(target) /
 *                                   backOff(inches) / nudge(target)
 *   intake:                         intakeOn / intakeOff / reverse(sec) / isFull
 *   chaining / conditions:          ifSeen(color).grab(color) / orElse / then /
 *                                   thenReturnTo / within / go
 *
 * A verb appends a step to a one-shot schedule. go(LinearOpMode) runs the
 * schedule to completion and owns the loop (for Pedro this includes
 * follower.update()); or pump start()/update()/abort() yourself from your own
 * loop, in which case YOU own the drivetrain's loop hook.
 *
 * Chains are ONE-SHOT: go()/update() leave the steps queued, so calling go()
 * again REPLAYS them. Build a chain, run it, then clear() before scheduling
 * the next one. Each step's timers (timeout, backOff deadline, tracker
 * last-seen) start when the step first ACTUALLY runs, not when it was built.
 *
 * then(other) moves the other wrangler's already-scheduled verbs onto this
 * chain; those steps stay bound to `other`, so their pickups are counted on
 * `other` and a same-wrangler orElse() fallback built against `other` would
 * append to `other`'s (now empty) chain - keep orElse/fallbacks on the
 * receiving wrangler in cross-chain builds.
 */
package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallChaseController;
import org.firstinspires.ftc.teamcode.BallTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public abstract class BallWrangler {

    public static final BallColor RED   = BallColor.RED;
    public static final BallColor BLUE  = BallColor.BLUE;
    public static final BallColor YELLOW = BallColor.YELLOW;

    /* ---- chase / approach tuning ----
     * CHASE_* mirrors BallChaseController (the single source of truth for the
     * no-odometry core), so a gain fixed in one is fixed in both. The wrapper
     * still re-exposes them as public statics: config systems can drive either
     * name. Wrapper-only verbs (align / approach / backOff / scan) keep their
     * own statics below. */
    public static double CHASE_STOP_DIST       = BallChaseController.STOP_DIST;
    public static double CHASE_AIM_TOL_DEG     = BallChaseController.AIM_TOL_DEG;
    public static double CHASE_DRIVE_MIN_TX    = BallChaseController.DRIVE_MIN_TX;
    public static double CHASE_DRIVE_KP        = BallChaseController.DRIVE_KP;
    public static double CHASE_MIN_FWD         = BallChaseController.MIN_FWD;
    public static double CHASE_MAX_FWD         = BallChaseController.MAX_FWD;
    public static double CHASE_TURN_KP         = BallChaseController.TURN_KP;
    public static double CHASE_MIN_TURN        = BallChaseController.MIN_TURN;
    public static double CHASE_MAX_TURN        = BallChaseController.MAX_TURN;
    public static double CHASE_COAST_MAX_DIST  = BallChaseController.COAST_MAX_DIST;
    public static double CHASE_COAST_POWER     = BallChaseController.COAST_POWER;
    public static long   CHASE_COAST_MS        = BallChaseController.COAST_MS;
    public static long   CHASE_PICKUP_MS       = BallChaseController.PICKUP_DWELL_MS;
    public static double CHASE_INTAKE_POWER    = BallChaseController.INTAKE_POWER;
    public static double SEARCH_TURN           = BallChaseController.SEARCH_TURN;
    public static long   CHASE_LOST_MS         = 500;   // wrapper-only: how long a lost ball is chaseable
    public static double GRAB_TIMEOUT_SEC      = 6.0;   // wrapper-only
    public static int    GRAB_FAIL_LIMIT       = 2;     // consecutive failed chases before a gather gives up

    public static double APPROACH_DIST         = 28.0;
    public static double NUDGE_DIST            = 12.0;

    public static double ALIGN_TOL_DEG         = 4.0;
    public static double ALIGN_TIMEOUT_SEC     = 2.5;
    public static double ALIGN_TURN_KP         = 0.05;

    public static double SCAN_SWEEP_DEG        = 120.0;
    public static double SCAN_TIMEOUT_SEC      = 3.0;

    public static double BACK_POWER            = 0.35;
    public static double MAX_FWD_IN_PER_SEC    = 18.0;

    public static double SCORE_OFF_AXIS_PENALTY = 0.25;  // per OFF_AXIS_SCALE_DEG of |tx|
    public static double SCORE_CONF_BONUS       = 0.20;  // up to -20% range for max confidence
    public static double OFF_AXIS_SCALE_DEG     = 45.0;

    public static double GO_TO_TIMEOUT_SEC      = 8.0;   // pose verbs (searchAt / then / thenReturnTo)

    public static long LOOP_MS = 10;   // go() loop period

    /** Pluggable full-intake detector; grabAll()/grabUpTo() stop when this says full. */
    public interface FullSensor {
        boolean isFull();
    }

    /** Optional hardware confirmation that a ball actually entered the intake
     *  (beam break, intake current spike). When set, a chase only counts the
     *  pickup - and reports won() - if confirmed() is true the moment the
     *  pickup dwell ends; otherwise the step fails cleanly (won = false, no
     *  pickup counted) so an orElse() fallback or a gather's failure cap can
     *  react instead of trusting a phantom. With no sensor, behavior is
     *  unchanged (optimistic count). */
    public interface PickupSensor {
        boolean confirmed();
    }

    /* ---------------- perception ---------------- */

    /** A drivetrain-free snapshot of one confident detection. subclasses and the
     *  WrapperLogicTest feed these in; the default reads the Limelight. */
    protected static final class Det {
        final int classId;
        final double bearingDeg, elevDeg, confidence, area;
        Det(int classId, double bearingDeg, double elevDeg, double confidence, double area) {
            this.classId = classId;
            this.bearingDeg = bearingDeg;
            this.elevDeg = elevDeg;
            this.confidence = confidence;
            this.area = area;
        }
    }

    protected final BallTracker tracker;
    protected final DcMotor intake;         // may be null
    protected final FullSensor full;        // may be null

    private Set<Integer> huntAllowed = new HashSet<>(BallTracker.CLASSES_RED_BLUE);
    private PickupSensor pickupSensor = null;

    /* ---------------- chain state ---------------- */

    private final ArrayList<Step> chain = new ArrayList<>();
    private final HashSet<Step> entered = new HashSet<>();
    private int pos = 0;
    private boolean running = false;
    private boolean finished = false;
    private Step last = null;
    private String lastError = null;

    private final ElapsedTime budget = new ElapsedTime();
    private double budgetSec = 0;
    private int pickups = 0;

    /* ---------------- steps ---------------- */

    private interface Step {
        void onEnter();
        boolean frame();          // true when this step is complete
        boolean won();            // reached its goal (vs failed / not-found)
        Runnable fallback();
        void setFallback(Runnable r);
        double timeoutSec();
        void setTimeout(double s);
    }

    private abstract class AStep implements Step {
        protected final ElapsedTime t0 = new ElapsedTime();
        private double timeout = 0;
        private Runnable fallback = null;
        protected String err = null;

        public void enterOnce() { t0.reset(); onEnter(); }
        public boolean timedOut() { return timeout > 0 && t0.seconds() > timeout; }
        @Override public double timeoutSec() { return timeout; }
        @Override public void setTimeout(double s) { timeout = s; }
        @Override public Runnable fallback() { return fallback; }
        @Override public void setFallback(Runnable r) { fallback = r; }

        protected void drive(double fwd, double strafe, double turnCw) {
            driveRobot(fwd, strafe, turnCw);
        }

        protected void intakePower(double p) {
            setIntake(p);
        }
    }

    private void setIntake(double p) {
        onIntake(p);
        if (intake != null) intake.setPower(p);
    }

    /** Observed every time an intake power is commanded (including 0). Hook for
     *  tests and subclasses that want to watch (not veto) intake commands. */
    protected void onIntake(double power) {}

    /** Chase one ball to the intake (or to a standoff for approach/nudge).
     *  resolve() picks the target the moment the step starts; chaseThen locks the
     *  tracker onto exactly that ball so two-ball flip-flop stays off. */
    private class ChaseStep extends AStep {
        private final Supplier<Target> resolve;
        private final boolean intakeDuring;
        private final double stopDist;
        private final boolean dwellPickup;
        private final boolean accountPickup;

        private Target target;
        private boolean won = false;
        private boolean have = false;
        private boolean dwellRunning = false;
        private double lastSeenDist = Double.MAX_VALUE;
        private final ElapsedTime lastSeen = new ElapsedTime();
        private final ElapsedTime dwell = new ElapsedTime();

        ChaseStep(Supplier<Target> resolve, boolean intakeDuring, double stopDist,
                  boolean dwellPickup, boolean accountPickup) {
            this.resolve = resolve;
            this.intakeDuring = intakeDuring;
            this.stopDist = stopDist;
            this.dwellPickup = dwellPickup;
            this.accountPickup = accountPickup;
            setTimeout(GRAB_TIMEOUT_SEC);
        }

        @Override public void onEnter() {
            target = resolve == null ? null : resolve.get();
            won = false;
            have = false;
            dwellRunning = false;
            lastSeenDist = Double.MAX_VALUE;
            lastSeen.reset();
            if (target == null) return;
            tracker.reset();
            tracker.addAllowedClass(target.color.classId);
            tracker.lockOn(target.color.classId, target.bearingDeg, target.elevDeg, target.confidence);
        }

        @Override public boolean frame() {
            if (won) return true;
            if (target == null || timedOut()) {
                drive(0, 0, 0);
                intakePower(0);
                return true;
            }

            BallTracker.Sighting s = trackerUpdate();
            if (s == null) {
                if (have && lastSeenDist <= CHASE_COAST_MAX_DIST) return coastOrArrive();
                if (lastSeen.milliseconds() > CHASE_LOST_MS) return giveUp();
                drive(0, 0, 0);
                return false;
            }
            if (s.predicted) {
                if (lastSeenDist <= CHASE_COAST_MAX_DIST) return coastOrArrive();
                double turn = Range.clip(s.txDeg * CHASE_TURN_KP, -CHASE_MAX_TURN, CHASE_MAX_TURN);
                if (Math.abs(turn) < CHASE_MIN_TURN) turn = Math.copySign(CHASE_MIN_TURN, s.txDeg);
                drive(0, 0, turn);
                return false;
            }

            lastSeen.reset();
            lastSeenDist = s.distIn;
            have = true;

            boolean aimed = Math.abs(s.txDeg) < CHASE_AIM_TOL_DEG;
            if (s.distIn <= stopDist && aimed) {
                drive(0, 0, 0);
                return arrive();
            }

            double turn = Range.clip(s.txDeg * CHASE_TURN_KP, -CHASE_MAX_TURN, CHASE_MAX_TURN);
            if (!aimed && Math.abs(turn) < CHASE_MIN_TURN) turn = Math.copySign(CHASE_MIN_TURN, s.txDeg);

            double fwd = 0.0;
            if (s.distIn > stopDist && Math.abs(s.txDeg) < CHASE_DRIVE_MIN_TX) {
                fwd = Range.clip((s.distIn - stopDist) * CHASE_DRIVE_KP, CHASE_MIN_FWD, CHASE_MAX_FWD);
            }
            intakePower(intakeDuring ? CHASE_INTAKE_POWER : 0);
            drive(fwd, 0, turn);
            return false;
        }

        /** Ball is at the intake: dwell the intake if asked, else stop clean. */
        private boolean arrive() {
            drive(0, 0, 0);
            if (!dwellPickup) {
                intakePower(0);
                succeed();
                return true;
            }
            if (!dwellRunning) { dwellRunning = true; dwell.reset(); }
            intakePower(intakeDuring ? CHASE_INTAKE_POWER : 0);
            if (dwell.milliseconds() >= CHASE_PICKUP_MS) {
                intakePower(0);
                succeed();
                return true;
            }
            return false;
        }

        /** Ball vanished right at the intake: coast, then treat it as picked. */
        private boolean coastOrArrive() {
            if (lastSeen.milliseconds() < CHASE_COAST_MS) {
                intakePower(intakeDuring ? CHASE_INTAKE_POWER : 0);
                drive(CHASE_COAST_POWER, 0, 0);
                return false;
            }
            drive(0, 0, 0);
            return arrive();
        }

        private boolean giveUp() {
            drive(0, 0, 0);
            intakePower(0);
            return true;
        }

        private void succeed() {
            drive(0, 0, 0);
            have = false;
            boolean confirmed = !accountPickup || pickupSensor == null || pickupSensor.confirmed();
            won = confirmed;
            if (accountPickup && confirmed) pickups++;
        }

        @Override public boolean won() { return won; }
    }

    /** grabAll() / grabUpTo(n): keep chasing balls until nothing is left, the
     *  intake is full, the count is reached, or a ball keeps failing to arrive
     *  (GRAB_FAIL_LIMIT consecutive timed-out/lost chases - stops the retry
     *  loop instead of chasing an unreachable ball forever). Each ball gets its
     *  own lock. won() = collected at least one, so grabAll().orElse(...) fires
     *  when the gather found nothing at all. */
    private class GatherStep extends AStep {
        private final int upTo;   // 0 = unlimited
        private int n = 0;
        private int consecutiveFails = 0;
        private ChaseStep inner = null;

        GatherStep(int upTo) {
            this.upTo = upTo;
        }

        @Override public void onEnter() {
            n = 0;
            consecutiveFails = 0;
            inner = null;
        }

        @Override public boolean frame() {
            if (upTo > 0 && n >= upTo) return true;
            if (isFull()) return true;
            if (inner != null) {
                if (!inner.frame()) return false;
                if (inner.won()) {
                    n++;
                    consecutiveFails = 0;
                } else if (++consecutiveFails >= GRAB_FAIL_LIMIT) {
                    return true;
                }
                inner = null;
                return false;
            }
            Target t = findNearest();
            if (t == null) return true;
            inner = new ChaseStep(() -> t, true, CHASE_STOP_DIST, true, true);
            inner.enterOnce();
            return false;
        }

        @Override public boolean won() { return n > 0; }
    }

    /** Rotate in place until a ball shows up, the sweep is covered, or the clock runs out. */
    private class ScanStep extends AStep {
        private final BallColor color;       // null = any allowed color
        private final boolean clockwise;
        private double startHeading;

        ScanStep(BallColor color, boolean clockwise) {
            this.color = color;
            this.clockwise = clockwise;
            if (timeoutSec() <= 0) setTimeout(SCAN_TIMEOUT_SEC);
        }

        @Override public void onEnter() {
            startHeading = headingRad();
        }

        @Override public boolean frame() {
            if (timedOut()) { drive(0, 0, 0); return true; }
            if (seen())     { drive(0, 0, 0); return true; }
            double swept = clockwise ? cwDeltaSince(startHeading) : -cwDeltaSince(startHeading);
            if (swept >= Math.toRadians(SCAN_SWEEP_DEG)) {
                drive(0, 0, 0);
                return true;
            }
            drive(0, 0, clockwise ? SEARCH_TURN : -SEARCH_TURN);
            return false;
        }

        @Override public boolean won() {
            return !timedOut() && seen();
        }

        private boolean seen() {
            return color != null ? canSee(color) : canSee();
        }
    }

    /** Turn in place until the chosen ball is nearly centered, or give up. */
    private class AlignStep extends AStep {
        private final Supplier<Target> resolve;
        private Target target;
        private boolean aimed = false;

        AlignStep(Supplier<Target> resolve) {
            this.resolve = resolve;
            setTimeout(ALIGN_TIMEOUT_SEC);
        }

        @Override public void onEnter() {
            target = resolve == null ? null : resolve.get();
            aimed = false;
            if (target == null) return;
            tracker.reset();
            tracker.addAllowedClass(target.color.classId);
            tracker.lockOn(target.color.classId, target.bearingDeg, target.elevDeg, target.confidence);
        }

        @Override public boolean frame() {
            if (target == null || timedOut()) { drive(0, 0, 0); return true; }
            BallTracker.Sighting s = trackerUpdate();
            if (s == null) { drive(0, 0, 0); return true; }
            if (Math.abs(s.txDeg) < ALIGN_TOL_DEG) {
                aimed = true;
                drive(0, 0, 0);
                return true;
            }
            double turn = Range.clip(s.txDeg * ALIGN_TURN_KP, -CHASE_MAX_TURN, CHASE_MAX_TURN);
            if (Math.abs(turn) < CHASE_MIN_TURN) turn = Math.copySign(CHASE_MIN_TURN, s.txDeg);
            drive(0, 0, turn);
            return false;
        }

        @Override public boolean won() { return aimed; }
    }

    /** Reverse straight for a set distance, intake off (backs the robot out after
     *  a grab/jam). The intake is zeroed on entry so an earlier intakeOn() cannot
     *  keep pulling through the back-off. */
    private class BackStep extends AStep {
        private final double inches;
        private double deadlineSec = 0;

        BackStep(double inches) {
            this.inches = inches;
        }

        @Override public void onEnter() {
            intakePower(0);
            double rate = Math.max(0.01, BACK_POWER * MAX_FWD_IN_PER_SEC);
            deadlineSec = inches / rate;
        }

        @Override public boolean frame() {
            if (t0.seconds() >= deadlineSec) { drive(0, 0, 0); return true; }
            drive(-BACK_POWER, 0, 0);
            return false;
        }

        @Override public boolean won() { return t0.seconds() >= deadlineSec; }
    }

    /** Drive to a field pose (searchAt / then / thenReturnTo). Fails fast when
     *  the drivetrain has no localization; the chain simply moves on. */
    private class GoToStep extends AStep {
        private final RobotPose pose;
        private boolean arrived = false;

        GoToStep(RobotPose pose) {
            this.pose = pose;
            setTimeout(BallWrangler.GO_TO_TIMEOUT_SEC);
        }

        @Override public void onEnter() {
            if (!hasPose()) { err = "pose verbs need localization (PedroWrangler or a MecanumPoseRouter)"; return; }
            if (!goToPose(pose)) { err = "goTo not supported here"; return; }
        }

        @Override public boolean frame() {
            if (err != null) { drive(0, 0, 0); return true; }
            if (arrivedAtPose() || timedOut()) {
                stopPoseMove();
                drive(0, 0, 0);
                arrived = !timedOut();
                return true;
            }
            return false;
        }

        @Override public boolean won() { return err == null && arrived; }
    }

    private class IntakeStep extends AStep {
        private final double power;
        IntakeStep(double power) { this.power = power; }
        @Override public void onEnter() {}
        @Override public boolean frame() { intakePower(power); return true; }
        @Override public boolean won() { return true; }
    }

    /** reverse(sec): run the intake backwards to unjam. */
    private class ReverseStep extends AStep {
        private final double sec;
        ReverseStep(double sec) { this.sec = sec; }
        @Override public void onEnter() {}
        @Override public boolean frame() {
            intakePower(-CHASE_INTAKE_POWER);
            if (t0.seconds() >= sec) { intakePower(0); return true; }
            return false;
        }
        @Override public boolean won() { return t0.seconds() >= sec; }
    }

    /** ifSeen(color).grab(color): run a sub-chain only when a ball is visible;
     *  skip cleanly otherwise. A skipped condition reports !won() so an
     *  .orElse() fallback can fire - and so does a condition whose sub-steps
     *  RAN but failed (e.g. the grab gave up or timed out): won() requires every
     *  sub-step to have finished AND succeeded. */
    private class ConditionalStep extends AStep {
        private final Supplier<Boolean> test;
        private final Supplier<List<Step>> build;
        private List<Step> sub = null;
        private int subPos = 0;
        private boolean allWon = true;

        ConditionalStep(Supplier<Boolean> test, Supplier<List<Step>> build) {
            this.test = test;
            this.build = build;
        }

        @Override public void onEnter() {
            allWon = true;
            sub = (test.get() == Boolean.TRUE) ? build.get() : null;
            subPos = 0;
        }

        @Override public boolean frame() {
            if (sub == null) return true;
            while (subPos < sub.size()) {
                Step s = sub.get(subPos);
                enterStep(s);
                if (!s.frame()) return false;
                if (!s.won()) allWon = false;
                if (s instanceof AStep && ((AStep) s).err != null) err = ((AStep) s).err;
                subPos++;
            }
            return true;
        }

        @Override public boolean won() { return sub != null && subPos >= sub.size() && allWon; }
    }

    /* ---------------- constructors ---------------- */

    public BallWrangler(Limelight3A limelight, DcMotor intakeOrNull) {
        this(new BallTracker(limelight), intakeOrNull, null);
    }

    public BallWrangler(Limelight3A limelight, DcMotor intakeOrNull, FullSensor fullOrNull) {
        this(new BallTracker(limelight), intakeOrNull, fullOrNull);
    }

    public BallWrangler(BallTracker tracker, DcMotor intakeOrNull, FullSensor fullOrNull) {
        this.tracker = tracker;
        this.intake = intakeOrNull;
        this.full = fullOrNull;
        tracker.setAllowedClasses(huntAllowed);
    }

    /* ---------------- subclass motion hooks ---------------- */

    protected boolean hasPose() { return false; }
    protected RobotPose robotPose() { return null; }
    protected boolean goToPose(RobotPose p) { return false; }
    protected boolean arrivedAtPose() { return true; }
    protected void stopPoseMove() {}
    protected double headingRad() { return 0; }
    protected double cwDeltaSince(double startHeadingRad) { return 0; }
    protected void loopHook() {}
    protected abstract void driveRobot(double fwd, double strafe, double turnCw);

    /** Poll the tracker once per chase/align frame. Override in tests to feed
     *  scripted sightings without a real Limelight. */
    protected BallTracker.Sighting trackerUpdate() {
        return tracker.update();
    }

    /* ---------------- color focus (which balls count as "allowed") ---------------- */

    public BallWrangler reds()      { setHunt(BallTracker.CLASS_RED); return this; }
    public BallWrangler blues()     { setHunt(BallTracker.CLASS_BLUE); return this; }
    public BallWrangler yellows()   { setHunt(BallTracker.CLASS_YELLOW_NEUTRAL); return this; }
    public BallWrangler alliance()  { setHunt(BallTracker.CLASS_RED, BallTracker.CLASS_BLUE); return this; }
    public BallWrangler everything(){ setHunt(BallTracker.CLASS_RED, BallTracker.CLASS_BLUE, BallTracker.CLASS_YELLOW_NEUTRAL); return this; }
    public BallWrangler setColors(Integer... ids) { setHunt(ids); return this; }

    private void setHunt(Integer... ids) {
        huntAllowed = new HashSet<>(Arrays.asList(ids));
        tracker.setAllowedClasses(huntAllowed);
    }

    /* ---------------- finding (look, don't drive) ---------------- */

    public boolean canSee() { return !findList(null).isEmpty(); }
    public boolean canSee(BallColor color) { return !findList(color).isEmpty(); }

    public int count() { return findList(null).size(); }
    public int count(BallColor color) { return findList(color).size(); }

    public Target findNearest()    { return best(null, 0); }
    public Target find(BallColor c) { return best(c, 0); }
    public Target findLeftmost()   { return best(null, 1); }
    public Target findRightmost()  { return best(null, 2); }
    public Target findBiggest()    { return best(null, 3); }
    public Target findBestScore()  { return best(null, 4); }

    /** find-style verbs, but with the returned Target field-positioned when the
     *  drivetrain has localization. */
    public Target findNearestAtPose() { return withPose(findNearest()); }

    /** Current confident detections from the Limelight, wrapped in Dets. Overridable for tests. */
    protected List<Det> rawDetections() {
        List<BallTracker.RawDet> dets = tracker.getConfidentResults();
        List<Det> out = new ArrayList<>();
        if (dets == null) return out;
        for (BallTracker.RawDet d : dets) {
            out.add(new Det(d.classId, d.txDeg, d.tyDeg, d.confidence, d.area));
        }
        return out;
    }

    /** Pick the best detection by metric: 0 nearest (lowest ty), 1 leftmost,
     *  2 rightmost, 3 biggest (area, else nearest), 4 best score. Null if none. */
    private Target best(BallColor color, int metric) {
        List<Det> dets = findList(color);
        if (dets.isEmpty()) return null;
        Det best = null;
        double bestV = -Double.MAX_VALUE;
        for (Det d : dets) {
            double v;
            switch (metric) {
                case 1: v = -d.bearingDeg; break;                 // most-negative tx
                case 2: v =  d.bearingDeg; break;                 // most-positive tx
                case 3: v =  d.area > 0 ? d.area : 1.0 / distOf(d); break;
                case 4: v = -scoreOf(distOf(d), d.bearingDeg, d.confidence); break;
                default: v = -distOf(d); break;                   // nearest
            }
            if (best == null || v > bestV) { bestV = v; best = d; }
        }
        return toTarget(best);
    }

    private List<Det> findList(BallColor color) {
        List<Det> out = new ArrayList<>();
        for (Det d : rawDetections()) {
            if (color != null) {
                if (d.classId == color.classId) out.add(d);
            } else if (huntAllowed.contains(d.classId)) {
                out.add(d);
            }
        }
        return out;
    }

    private Target toTarget(Det d) {
        return new Target(BallColor.fromClassId(d.classId), d.bearingDeg, d.elevDeg,
                distOf(d), d.confidence, scoreOf(distOf(d), d.bearingDeg, d.confidence));
    }

    private double distOf(Det d) {
        return tracker.groundRange(d.elevDeg);
    }

    /** Lower is better: base range, nudged up as the ball sits further off the
     *  robot's forward axis and down as confidence rises. */
    public static double scoreOf(double distIn, double bearingDeg, double confidence) {
        double offAxis = Math.abs(bearingDeg) / OFF_AXIS_SCALE_DEG;
        double conf = Range.clip((confidence - BallTracker.MIN_CONF) / (1.0 - BallTracker.MIN_CONF), 0, 1);
        return distIn * (1.0 + SCORE_OFF_AXIS_PENALTY * offAxis) * (1.0 - SCORE_CONF_BONUS * conf);
    }

    private Target withPose(Target t) {
        if (t != null && hasPose() && robotPose() != null) return atField(t, robotPose());
        return t;
    }

    /** Project a camera sighting into field coords (same geometry as BallChaseFollower).
     *  The camera's mount offset (CAM_X_OFFSET forward, CAM_Y_OFFSET left, in
     *  inches) is rotated from the camera frame and added to the robot pose, so
     *  a camera not at robot center still produces field-accurate points. */
    public static double[] projectBall(double txDeg, double tyDeg, RobotPose pose) {
        final double p = Math.toRadians(BallTracker.CAM_PITCH_DEG);
        final double h = BallTracker.CAM_H - BallTracker.BALL_H;
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));
        double denom = Math.sin(p) - tanTy * Math.cos(p);
        if (denom <= 0.02) return null;
        double s = h / denom;
        double xr = s * (Math.cos(p) + tanTy * Math.sin(p));
        double yr = -s * tanTx;
        double th = pose.headingRad;
        double ox = BallTracker.CAM_X_OFFSET * Math.cos(th) - BallTracker.CAM_Y_OFFSET * Math.sin(th);
        double oy = BallTracker.CAM_X_OFFSET * Math.sin(th) + BallTracker.CAM_Y_OFFSET * Math.cos(th);
        return new double[]{
                pose.x + ox + xr * Math.cos(th) - yr * Math.sin(th),
                pose.y + oy + xr * Math.sin(th) + yr * Math.cos(th)};
    }

    public Target atField(Target t, RobotPose pose) {
        double[] f = projectBall(t.bearingDeg, t.elevDeg, pose);
        if (f == null) return t;
        return t.withField(f[0], f[1]);
    }

    /* ---------------- grabbing (drive and intake) ---------------- */

    public BallWrangler grabNearest()    { addChase(this::findNearest); return this; }
    public BallWrangler grab(BallColor c) { addChase(() -> find(c)); return this; }
    public BallWrangler grab(Target t)   { final Target tt = t; addChase(() -> tt); return this; }
    public BallWrangler grabLeftmost()   { addChase(this::findLeftmost); return this; }
    public BallWrangler grabRightmost()  { addChase(this::findRightmost); return this; }
    public BallWrangler grabAll()        { add(new GatherStep(0)); return this; }
    public BallWrangler grabUpTo(int n)  { add(new GatherStep(Math.max(1, n))); return this; }

    public BallWrangler grabTwo(BallColor a, BallColor b) { grab(a); and(b); return this; }
    public BallWrangler and(BallColor color) { return grab(color); }

    private void addChase(Supplier<Target> resolve) {
        add(new ChaseStep(resolve, true, CHASE_STOP_DIST, true, true));
    }

    private ChaseStep buildChase(Supplier<Target> resolve, boolean intakeDuring,
                                 double stopDist, boolean dwell, boolean account) {
        return new ChaseStep(resolve, intakeDuring, stopDist, dwell, account);
    }

    /* ---------------- searching ---------------- */

    public BallWrangler scan()          { add(new ScanStep(null, true)); return this; }
    public BallWrangler scanLeft()      { add(new ScanStep(null, false)); return this; }
    public BallWrangler scanRight()     { add(new ScanStep(null, true)); return this; }
    public BallWrangler lookFor(BallColor color) { add(new ScanStep(color, true)); return this; }

    public BallWrangler searchAt(RobotPose pose) {
        then(pose);
        scan();
        return this;
    }

    /* ---------------- moving around a ball ---------------- */

    public BallWrangler approach(Target t) {
        final Target tt = t;
        add(buildChase(() -> tt, false, APPROACH_DIST, false, false));
        return this;
    }

    public BallWrangler alignTo(Target t) {
        final Target tt = t;
        add(new AlignStep(() -> tt));
        return this;
    }

    public BallWrangler alignTo(BallColor c) {
        add(new AlignStep(() -> find(c)));
        return this;
    }

    public BallWrangler nudge(Target t) {
        final Target tt = t;
        add(buildChase(() -> tt, false, NUDGE_DIST, false, false));
        return this;
    }

    public BallWrangler backOff(double inches) {
        add(new BackStep(inches));
        return this;
    }

    /* ---------------- intake ---------------- */

    public BallWrangler intakeOn()  { add(new IntakeStep(CHASE_INTAKE_POWER)); return this; }
    public BallWrangler intakeOff() { add(new IntakeStep(0.0)); return this; }
    public BallWrangler reverse(double sec) { add(new ReverseStep(sec)); return this; }

    public boolean isFull() { return full != null && full.isFull(); }

    /** Optional hardware confirmation for pickups (beam break, current spike).
     *  See PickupSensor. Fluent. */
    public BallWrangler withPickupSensor(PickupSensor s) { pickupSensor = s; return this; }

    /* ---------------- chaining / conditions ---------------- */

    public When ifSeen(BallColor color) { return new When(color); }
    public When when(BallColor color)   { return new When(color); }

    /** The old-style `ifSeen(RED).grab(RED)` sugar. Every verb here is a thin
     *  wrapper around ConditionalStep (run this sub-chain when the color is
     *  visible, skip it otherwise, and report !won() when skipped or when the
     *  sub-chain ran but failed). Continue the chain straight off the When:
     *  within / then / thenReturnTo / go are delegated to the owning wrangler,
     *  so `ifSeen(RED).grab(RED).within(6).go(this)` reads as one fluent line. */
    public class When {
        private final BallColor color;
        When(BallColor color) { this.color = color; }
        public When grab(BallColor c) {
            add(new ConditionalStep(() -> canSee(color), () -> {
                List<Step> l = new ArrayList<>();
                l.add(buildChase(() -> find(c), true, CHASE_STOP_DIST, true, true));
                return l;
            }));
            return this;
        }
        public When approach(BallColor c) {
            add(new ConditionalStep(() -> canSee(color), () -> {
                List<Step> l = new ArrayList<>();
                l.add(buildChase(() -> find(c), false, APPROACH_DIST, false, false));
                return l;
            }));
            return this;
        }
        public When backOff(double inches) {
            add(new ConditionalStep(() -> canSee(color), () -> {
                List<Step> l = new ArrayList<>();
                l.add(new BackStep(inches));
                return l;
            }));
            return this;
        }
        public When orElse(Runnable fallback) {
            BallWrangler.this.orElse(fallback);
            return this;
        }

        /* ---- fluent continuations, delegated to the owning wrangler ---- */

        public When within(double seconds)   { BallWrangler.this.within(seconds); return this; }
        public When orGiveUp(double seconds) { BallWrangler.this.orGiveUp(seconds); return this; }
        public When then(RobotPose pose)     { BallWrangler.this.then(pose); return this; }
        public When then(BallWrangler other) { BallWrangler.this.then(other); return this; }
        public When thenReturnTo(RobotPose pose) { BallWrangler.this.then(pose); return this; }
        public int go(LinearOpMode op)       { return BallWrangler.this.go(op); }
    }

    /** `then(other)`: run `other`'s already-scheduled verbs right after this
     *  chain's, so sequences can span two wranglers (e.g. a teleop wrangler's
     *  grab followed by an auto wrangler's return-to-pose). `other` is drained -
     *  its steps move HERE, so calling `other.go()/update()` later does nothing.
     *  A same-wrangler call (`x.then(x.verb())`) stays a no-op confirm, since the
     *  verb was already queued on this chain. Call before either chain has
     *  started. */
    public BallWrangler then(BallWrangler other) {
        if (other == null || other == this) return this;
        if (other.running || other.finished || other.pos > 0)
            throw new IllegalStateException("then(): call before either chain starts");
        chain.addAll(other.chain);
        if (other.last != null) last = other.last;
        other.chain.clear();
        other.last = null;
        return this;
    }

    public BallWrangler then(RobotPose pose) { add(new GoToStep(pose)); return this; }
    public BallWrangler thenReturnTo(RobotPose pose) { return then(pose); }

    /** Time budget for the whole run - go()/update() stop and finish cleanly. */
    public BallWrangler within(double seconds) { budgetSec = seconds; return this; }

    /** Caps the step added by the most recent verb (e.g. lookFor(RED).orGiveUp(3)). */
    public BallWrangler orGiveUp(double seconds) {
        if (last != null) last.setTimeout(seconds);
        return this;
    }

    /** On the preceding step failing to find/do anything, run this fallback
     *  instead of whatever comes next (it runs before the rest of the chain). */
    public BallWrangler orElse(Runnable fallback) {
        if (last != null) last.setFallback(fallback);
        return this;
    }

    /* ---------------- step bookkeeping ---------------- */

    private void add(Step s) {
        chain.add(s);
        last = s;
    }

    private void enterStep(Step s) {
        if (!entered.add(s)) return;
        if (s instanceof AStep) ((AStep) s).t0.reset();
        s.onEnter();
    }

    private void runFallback(Runnable r) {
        int mark = chain.size();
        r.run();
        if (chain.size() <= mark) return;
        List<Step> fb = new ArrayList<>(chain.subList(mark, chain.size()));
        chain.subList(mark, chain.size()).clear();
        chain.addAll(Math.min(pos + 1, chain.size()), fb);
    }

    /* ---------------- run it ---------------- */

    /** Drop every scheduled step and reset the wrangler to idle, so it can be
     *  reused for a fresh chain (chains are one-shot: go()/update() never clear
     *  them, and start() re-runs what is still queued). */
    public BallWrangler clear() {
        chain.clear();
        entered.clear();
        pos = 0;
        running = false;
        finished = false;
        last = null;
        budgetSec = 0;
        lastError = null;
        return this;
    }

    /** Begin a fresh run of whatever is in the chain. The chain itself survives
     *  (call clear() first to rebuild a one-shot schedule on a reused wrangler). */
    public void start() {
        pos = 0;
        running = true;
        finished = false;
        pickups = 0;
        entered.clear();
        budget.reset();
        tracker.reset();
    }

    /** Drive one step forward. Returns true when the chain has finished. */
    public boolean update() {
        if (!running) return isDone();
        if (budgetSec > 0 && budget.seconds() > budgetSec) { finished(); return true; }
        while (running && pos < chain.size()) {
            Step s = chain.get(pos);
            enterStep(s);
            if (!s.frame()) return isDone();
            if (s instanceof AStep && ((AStep) s).err != null) lastError = ((AStep) s).err;
            boolean ok = s.won();
            if (!ok && s.fallback() != null) runFallback(s.fallback());
            pos++;
        }
        finished();
        return true;
    }

    /** Stop everything: wheels zeroed, intake off, back to idle. */
    public void abort() {
        running = false;
        finished = true;
        stopPoseMove();
        driveRobot(0, 0, 0);
        setIntake(0);
    }

    private void finished() {
        running = false;
        finished = true;
        stopPoseMove();
        driveRobot(0, 0, 0);
        setIntake(0);
    }

    /** Blocking convenience: run the chain, own the loop (including the Pedro
     *  follower), stream telemetry, abort on opMode stop, return balls picked. */
    public int go(LinearOpMode op) {
        start();
        while (op.opModeIsActive() && !finished) {
            loopHook();
            update();
            addTelemetry(op.telemetry);
            op.telemetry.update();
            try {
                Thread.sleep(LOOP_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        abort();
        return getPickups();
    }

    /* ---------------- status ---------------- */

    public boolean isDone()   { return finished; }
    public boolean isActive() { return running; }
    public int getPickups()   { return pickups; }
    public int got()          { return pickups; }
    public String lastError() { return lastError; }
    public BallColor colorOf(Target t) { return t == null ? null : t.color; }

    /** Number of steps still scheduled (before go(), the whole chain; during a
     *  run, only what is left). Useful in telemetry and cross-chain tests. */
    public int scheduled() {
        return Math.max(0, chain.size() - (running ? pos : 0));
    }

    public void addTelemetry(Telemetry t) {
        t.addData("wrangler", "%s  picked=%d  scheduled=%d  budget=%s",
                running ? "running" : (finished ? "done" : "idle"),
                pickups, scheduled(),
                budgetSec <= 0 ? "none" : String.format("%.0fs", budgetSec));
        Target near = canSee() ? findNearest() : null;
        t.addData("wrangler seen", near == null ? "none" : near.toString());
        if (lastError != null) t.addData("wrangler note", lastError);
    }

    /* ---------------- helpers ---------------- */

    protected static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}