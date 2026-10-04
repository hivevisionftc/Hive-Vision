/*
 * WrapperLogicTest - desktop self-test for the verb wrapper. Runs the pure
 * logic (color mapping, scoring, find/count selection, field projection, chain
 * semantics: timeouts, fallbacks, conditional skips, cross-wrangler then(),
 * gather/pickup counting, full-sensor stop, time budget, pose-fail fast path,
 * intake/reverse verbs) through a hardware-free subclass feeding scripted
 * sightings - no Limelight, no motors. Compile with the real tree and run:
 *
 *     java -cp <compile-dir> org.firstinspires.ftc.teamcode.wrapper.WrapperLogicTest
 */
package org.firstinspires.ftc.teamcode.wrapper;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.BallTracker;

import java.util.ArrayList;
import java.util.List;

public class WrapperLogicTest extends BallWrangler {

    /** Records every drive command so tests can assert what actually got driven. */
    static class DriveLog {
        final double fwd, strafe, turn;
        DriveLog(double fwd, double strafe, double turn) { this.fwd = fwd; this.strafe = strafe; this.turn = turn; }
    }

    private List<Det> dets = new ArrayList<>();
    final List<DriveLog> drives = new ArrayList<>();
    final List<Double> intakeLog = new ArrayList<>();

    /** When true, trackerUpdate() returns `scripted` instead of the (null) tracker. */
    boolean scriptedSighting = false;
    BallTracker.Sighting scripted = null;

    /** When true, isFull() reports the intake is full (stops gathers). */
    boolean simulateFull = false;

    private WrapperLogicTest() {
        super(new BallTracker((Limelight3A) null), null, null);
    }

    @Override protected List<Det> rawDetections() { return dets; }
    @Override protected void driveRobot(double fwd, double strafe, double turn) {
        drives.add(new DriveLog(fwd, strafe, turn));
    }
    @Override protected void onIntake(double power) { intakeLog.add(power); }
    @Override protected BallTracker.Sighting trackerUpdate() {
        return scriptedSighting ? scripted : super.trackerUpdate();
    }
    @Override public boolean isFull() { return simulateFull; }
    @Override protected double headingRad() { return 0; }
    @Override protected double cwDeltaSince(double start) { return 0; }

    private void setDets() {
        dets = new ArrayList<>();
    }
    private void addRed(double tx, double ty, double conf, double area) {
        dets.add(new Det(BallTracker.CLASS_RED, tx, ty, conf, area));
    }
    private void addBlue(double tx, double ty, double conf, double area) {
        dets.add(new Det(BallTracker.CLASS_BLUE, tx, ty, conf, area));
    }

    private static void pump(WrapperLogicTest w, long ms) {
        w.start();
        long deadline = System.currentTimeMillis() + ms;
        while (!w.isDone() && System.currentTimeMillis() < deadline) {
            w.update();
            try { Thread.sleep(1); } catch (InterruptedException e) { Thread.currentThread().interrupt(); break; }
        }
    }

    private static int failures = 0;
    private static void check(boolean ok, String what) {
        System.out.println((ok ? "  [ok] " : "  [FAIL] ") + what);
        if (!ok) failures++;
    }

    public static void main(String[] args) {
        testColorMapping();
        testScoring();
        testProjection();
        testFindAndCount();
        testFallbackChain();
        testConditionalSkip();
        testGather();
        testBudget();
        testGoToError();
        testIntakeVerbs();
        testCrossChain();
        testTimersStartOnEnter();
        testClear();
        testScanLeft();
        testCondFailOrElse();
        testGatherOrElse();
        testWhenFluent();
        testGatherFailCap();
        testCrossChainPickups();
        testTrackerLock();
        testTrackerScanGating();

        if (failures > 0) {
            System.out.println("wrapper self-test FAILED: " + failures + " assertion(s)");
            System.exit(1);
        }
        System.out.println("wrapper self-test OK: colors, scoring, projection, find/count, chain "
                + "semantics, gather, budget, pose-fail, intake, cross-chain, timers, clear, "
                + "scan-sweep, cond-fail, gather-orElse, When-fluent, gather-cap, "
                + "cross-chain pickups, tracker lock, scan gating");
    }

    private static void testColorMapping() {
        System.out.println("color mapping");
        check(BallColor.RED.classId == BallTracker.CLASS_RED, "RED -> class " + BallTracker.CLASS_RED);
        check(BallColor.BLUE.classId == BallTracker.CLASS_BLUE, "BLUE -> class " + BallTracker.CLASS_BLUE);
        check(BallColor.YELLOW.classId == BallTracker.CLASS_YELLOW_NEUTRAL, "YELLOW -> class 0");
        check(BallColor.fromClassId(1) == BallColor.RED, "fromClassId(1) == RED");
        check(BallColor.fromClassId(99) == null, "fromClassId(99) == null");
    }

    private static void testScoring() {
        System.out.println("scoring");
        check(BallWrangler.scoreOf(50, 0, 0.9) < BallWrangler.scoreOf(70, 0, 0.9), "closer ball scores better");
        check(BallWrangler.scoreOf(50, 40, 0.9) > BallWrangler.scoreOf(50, 0, 0.9), "off-axis ball penalized");
        check(BallWrangler.scoreOf(50, 0, 0.44) > BallWrangler.scoreOf(50, 0, 0.9), "low confidence penalized");
    }

    private static void testProjection() {
        System.out.println("field projection");
        RobotPose origin = new RobotPose(0, 0, 0);
        double[] p = BallWrangler.projectBall(0, 0, origin);
        double expected = BallTracker.CAM_H - BallTracker.BALL_H;
        double dist = new BallTracker((Limelight3A) null).groundRange(0);
        check(p != null && Math.abs(p[0] - dist) < 0.01, String.format(
                "forward-axis ball -> (%.2f, %.2f), expected dist %.2f", p == null ? -1 : p[0], p == null ? -1 : p[1], dist));

        RobotPose r = new RobotPose(0, 0, 0);
        double[] q = BallWrangler.projectBall(12, 8, r);
        double[] rot = BallWrangler.projectBall(12, 8, new RobotPose(0, 0, Math.toRadians(30)));
        double magQ = Math.hypot(q[0], q[1]), magR = Math.hypot(rot[0], rot[1]);
        check(Math.abs(magQ - magR) < 0.01, "rotation preserves projected range (" + magQ + " ~ " + magR + ")");
    }

    private static void testFindAndCount() {
        System.out.println("find / count");
        WrapperLogicTest w = new WrapperLogicTest();
        w.addRed(-5, 10, 0.9, 0.60);   // far red, BIGGEST blob
        w.addRed( 3,  3, 0.8, 0.20);   // nearest red
        w.addBlue(-8,  8, 0.75, 0.10);

        check(w.canSee(), "canSee() any ball");
        check(w.canSee(BallColor.RED), "canSee(RED)");
        check(!w.canSee(BallColor.YELLOW), "no yellows visible");
        check(w.count() == 3, "count() == 3");
        check(w.count(BallColor.RED) == 2 && w.count(BallColor.BLUE) == 1, "count(RED)=2 count(BLUE)=1");

        Target near = w.findNearest();
        check(near != null && near.color == BallColor.RED && Math.abs(near.distIn - w.tracker.groundRange(3)) < 0.01,
                "findNearest picks the closest (ty=3) red");

        Target r = w.find(BallColor.BLUE);
        check(r != null && r.color == BallColor.BLUE && Math.abs(r.distIn - w.tracker.groundRange(8)) < 0.01, "find(BLUE)");

        check(w.findLeftmost().bearingDeg == -8, "findLeftmost picks tx=-8");
        check(w.findRightmost().bearingDeg == 3, "findRightmost picks tx=+3");
        Target big = w.findBiggest();
        check(big != null && Math.abs(big.distIn - w.tracker.groundRange(10)) < 0.01,
                "findBiggest picks the far, large-area blob instead of the nearest");

        Target bestScore = w.findBestScore();
        check(bestScore != null && bestScore.color == BallColor.RED, "findBestScore favors nearest red");
    }

    private static void testFallbackChain() {
        System.out.println("chain: orGiveUp + orElse fallback");
        WrapperLogicTest w = new WrapperLogicTest();
        w.scan().orGiveUp(0.005).orElse(() -> w.backOff(0.1));
        pump(w, 1000);
        boolean droveBack = false;
        for (DriveLog d : w.drives) if (d.fwd < 0) droveBack = true;
        check(w.isDone(), "chain finished");
        check(w.getPickups() == 0, "no ball was picked");
        check(droveBack, "failed scan ran the orElse backOff fallback");
    }

    private static void testConditionalSkip() {
        System.out.println("chain: ifSeen(RED).grab(RED).orElse(...) skip");
        WrapperLogicTest w = new WrapperLogicTest();
        w.ifSeen(RED).grab(RED).orElse(() -> w.backOff(0.1));
        pump(w, 1000);
        boolean droveBack = false;
        for (DriveLog d : w.drives) if (d.fwd < 0) droveBack = true;
        check(w.isDone(), "conditional chain finished");
        check(droveBack, "no red -> skipped grab, ran orElse backOff");
    }

    private static void testCrossChain() {
        System.out.println("chain: then(other) cross-wrangler chaining");
        WrapperLogicTest a = new WrapperLogicTest();
        WrapperLogicTest b = new WrapperLogicTest();
        b.backOff(0.1);
        b.reverse(0.005);
        a.scan().orGiveUp(0.005).then(b);
        pump(a, 1500);
        check(a.isDone(), "cross chain finished");
        check(b.scheduled() == 0, "other drained after then() (scheduled=" + b.scheduled() + ")");
        boolean droveBack = false;
        for (DriveLog d : b.drives) if (d.fwd < 0) droveBack = true;
        check(droveBack, "b's backOff ran inside a's chain");
        check(b.intakeLog.contains(-1.0), "b's reverse ran inside a's chain");
    }

    private static void testGather() {
        System.out.println("chain: grabUpTo / gather");
        long oldDwell = BallWrangler.CHASE_PICKUP_MS;
        BallWrangler.CHASE_PICKUP_MS = 20;              // shrink the pickup dwell for the test
        try {
            WrapperLogicTest w = new WrapperLogicTest();
            w.addRed(-10, -8, 0.9, 0.3);                // far red
            w.addRed(-2, -3, 0.85, 0.2);                // near red
            w.scriptedSighting = true;
            w.scripted = new BallTracker.Sighting(BallTracker.CLASS_RED, 0.4, -6.0, 0.9, false, 5.0);
            w.grabUpTo(2);
            pump(w, 3000);
            check(w.isDone(), "grabUpTo(2) finished");
            check(w.getPickups() == 2, "grabUpTo(2) picked both (got " + w.getPickups() + ")");

            WrapperLogicTest e = new WrapperLogicTest();
            e.scriptedSighting = true;
            e.scripted = new BallTracker.Sighting(BallTracker.CLASS_RED, 0, -6, 0.9, false, 5.0);
            e.grabUpTo(3);
            pump(e, 1000);
            check(e.isDone() && e.getPickups() == 0, "grabUpTo(3) with no balls -> 0 pickups, done");

            WrapperLogicTest f = new WrapperLogicTest();
            f.addRed(-4, -2, 0.9, 0.3);
            f.simulateFull = true;
            f.grabAll();
            pump(f, 1000);
            check(f.isDone() && f.getPickups() == 0, "full intake short-circuits grabAll");
        } finally {
            BallWrangler.CHASE_PICKUP_MS = oldDwell;
        }
    }

    private static void testBudget() {
        System.out.println("chain: within() time budget");
        WrapperLogicTest w = new WrapperLogicTest();
        w.within(0.05);
        w.scan();
        long t0 = System.currentTimeMillis();
        pump(w, 2000);
        long dt = System.currentTimeMillis() - t0;
        check(w.isDone(), "budgeted chain finished");
        check(dt >= 40 && dt < 1500, "budget enforced (finished in " + dt + " ms)");
    }

    private static void testGoToError() {
        System.out.println("chain: then(pose) fails fast without localization");
        WrapperLogicTest w = new WrapperLogicTest();
        w.then(new RobotPose(24, 24, Math.toRadians(90))).orElse(() -> w.backOff(0.1));
        pump(w, 1500);
        boolean droveBack = false;
        for (DriveLog d : w.drives) if (d.fwd < 0) droveBack = true;
        check(w.isDone(), "pose chain finished");
        check(w.lastError() != null && w.lastError().contains("localization"),
                "pose verb recorded lastError");
        check(droveBack, "pose failure ran the orElse backOff fallback");
    }

    private static void testIntakeVerbs() {
        System.out.println("verbs: intakeOn / reverse / intakeOff");
        WrapperLogicTest w = new WrapperLogicTest();
        w.intakeOn().reverse(0.005).intakeOff();
        pump(w, 1500);
        check(w.isDone(), "intake chain finished");
        check(w.intakeLog.contains(1.0), "intakeOn commanded +power");
        check(w.intakeLog.contains(-1.0), "reverse commanded negative power");
        check(w.intakeLog.contains(0.0), "intakeOff/finish zeroed the intake");
    }

    private static void testTimersStartOnEnter() {
        System.out.println("timers start when the step starts");
        WrapperLogicTest w = new WrapperLogicTest();
        w.backOff(0.5);                        // ~80 ms of driving, scheduled well ahead
        try { Thread.sleep(200); } catch (InterruptedException e) { return; }
        pump(w, 1000);
        int back = 0;
        for (DriveLog d : w.drives) if (d.fwd < 0) back++;
        check(back > 0, "backOff still drives after sitting scheduled (" + back + " frames)");
    }

    private static void testClear() {
        System.out.println("chain: clear() for reuse");
        WrapperLogicTest w = new WrapperLogicTest();
        w.backOff(0.1);
        w.intakeOn();
        pump(w, 1000);
        check(w.isDone(), "first run finished");
        w.clear();
        check(!w.isActive() && w.scheduled() == 0, "clear() empties the chain");
        pump(w, 200);
        check(w.isDone(), "empty chain finishes immediately");
    }

    /** Subclass that simulates real rotation: driveRobot turns feed an
     *  estimated heading, so scan()'s sweep logic can be exercised with no
     *  motors and no heading sensor (same sign conventions as MecanumWrangler:
     *  turn + = clockwise, heading CCW+). */
    static class RotHarness extends WrapperLogicTest {
        static final double TURN_RATE = 8.0, DT = 0.02;   // rad per power-sec, loop period
        double heading = 0;

        @Override protected void driveRobot(double fwd, double strafe, double turn) {
            drives.add(new DriveLog(fwd, strafe, turn));
            heading = BallWrangler.wrap(heading - turn * TURN_RATE * DT);
        }
        @Override protected double headingRad() { return heading; }
        @Override protected double cwDeltaSince(double start) { return -BallWrangler.wrap(heading - start); }
    }

    private static boolean droveReverse(WrapperLogicTest w) {
        for (DriveLog d : w.drives) if (d.fwd < 0) return true;
        return false;
    }

    /** Scripts detections + staleness straight into a BallTracker (laptop test
     *  of the lock/gating logic - no Limelight). */
    static final class ScriptedSource implements BallTracker.DetectionSource {
        List<BallTracker.RawDet> dets = new ArrayList<>();
        long staleness = 0;
        @Override public List<BallTracker.RawDet> latest() { return dets; }
        @Override public long stalenessMs() { return staleness; }
    }

    private static List<BallTracker.RawDet> detsOf(BallTracker.RawDet... items) {
        List<BallTracker.RawDet> l = new ArrayList<>();
        for (BallTracker.RawDet d : items) l.add(d);
        return l;
    }

    private static void testScanLeft() {
        System.out.println("scan: sweep limit ends the turn");
        RotHarness r = new RotHarness();
        r.scanLeft();
        long t0 = System.currentTimeMillis();
        pump(r, 1500);
        long dt = System.currentTimeMillis() - t0;
        check(r.isDone(), "scanLeft finished");
        check(dt < 1200,
                "scanLeft stopped at the 120 deg sweep, not the 3 s timeout (" + dt + " ms)");
    }

    private static void testCondFailOrElse() {
        System.out.println("chain: failed grab inside ifSeen triggers orElse");
        WrapperLogicTest w = new WrapperLogicTest();
        w.addRed(-4, -2, 0.9, 0.3);              // visible at the gate...
        w.ifSeen(RED).grab(RED).orElse(() -> w.backOff(0.1));   // ...but never reachable
        pump(w, 2000);
        check(w.isDone(), "failed conditional chain finished");
        check(droveReverse(w), "grab gave up inside ifSeen -> orElse fired");
    }

    private static void testGatherOrElse() {
        System.out.println("chain: grabAll().orElse fires when nothing collected");
        long oldDwell = BallWrangler.CHASE_PICKUP_MS;
        BallWrangler.CHASE_PICKUP_MS = 20;
        try {
            WrapperLogicTest w = new WrapperLogicTest();
            w.grabAll().orElse(() -> w.backOff(0.1));
            pump(w, 1000);
            check(w.isDone(), "empty gather finished");
            check(w.getPickups() == 0, "empty gather collected nothing");
            check(droveReverse(w), "grabAll().orElse ran when nothing was collected");
        } finally {
            BallWrangler.CHASE_PICKUP_MS = oldDwell;
        }
    }

    private static void testWhenFluent() {
        System.out.println("chain: When delegates within/then/thenReturnTo");
        WrapperLogicTest w = new WrapperLogicTest();
        WrapperLogicTest parked = new WrapperLogicTest();
        parked.backOff(0.1);
        w.addRed(-3, -3, 0.9, 0.2);
        w.ifSeen(RED).grab(RED).within(3).then(new RobotPose(10, 0, 0)).then(parked)
                .thenReturnTo(new RobotPose(0, 10, 0));
        pump(w, 5000);
        check(w.isDone(), "When-chained chain finished (delegated go/within/then compile + run)");
        check(parked.scheduled() == 0, "When.then(other) drained the other wrangler");
        check(droveReverse(parked), "When.then(other) ran other's steps in the chain");
    }

    private static void testGatherFailCap() {
        System.out.println("chain: gather gives up after GRAB_FAIL_LIMIT lost chases");
        WrapperLogicTest w = new WrapperLogicTest();
        w.addRed(-4, -2, 0.9, 0.3);            // found by findNearest but never visible to the tracker
        w.grabUpTo(5);
        long t0 = System.currentTimeMillis();
        pump(w, 5000);
        long dt = System.currentTimeMillis() - t0;
        check(w.isDone(), "gather ended (did not loop until the pump deadline)");
        check(w.getPickups() == 0, "nothing was collected");
        check(dt < 4000, "ended via GRAB_FAIL_LIMIT, quickly (" + dt + " ms)");
    }

    private static void testCrossChainPickups() {
        System.out.println("chain: then(other) pickup accounting stays on the source");
        long oldDwell = BallWrangler.CHASE_PICKUP_MS;
        BallWrangler.CHASE_PICKUP_MS = 20;
        try {
            WrapperLogicTest a = new WrapperLogicTest();
            WrapperLogicTest b = new WrapperLogicTest();
            b.addRed(-3, -3, 0.9, 0.2);
            b.scriptedSighting = true;
            b.scripted = new BallTracker.Sighting(BallTracker.CLASS_RED, 0.2, -5.0, 0.9, false, 5.0);
            b.grab(RED);
            a.then(b);
            pump(a, 3000);
            check(a.isDone(), "cross chain with a moved grab finished");
            check(b.getPickups() == 1, "moved step's pickup counts on its source (b got "
                    + b.getPickups() + ")");
            check(a.getPickups() == 0, "receiver's own pickups stay 0");
        } finally {
            BallWrangler.CHASE_PICKUP_MS = oldDwell;
        }
    }

    private static void testTrackerLock() {
        System.out.println("tracker: lock is color-bound");
        ScriptedSource src = new ScriptedSource();
        BallTracker t = new BallTracker(src);
        t.setAllowedClasses(BallTracker.CLASS_RED, BallTracker.CLASS_BLUE);

        src.dets = detsOf(new BallTracker.RawDet(BallTracker.CLASS_RED, -1.0, -7.0, 0.9, 0.3));
        BallTracker.Sighting s1 = t.update();
        check(s1 != null && s1.classId == BallTracker.CLASS_RED, "first lock adopts the visible red");

        // red drifts, and a BLUE appears 6 deg away - well inside LOCK_GATE_DEG
        src.dets = detsOf(new BallTracker.RawDet(BallTracker.CLASS_RED, -2.0, -7.5, 0.9, 0.3),
                          new BallTracker.RawDet(BallTracker.CLASS_BLUE, 6.0, -6.0, 0.9, 0.3));
        BallTracker.Sighting s2 = t.update();
        check(s2 != null && !s2.predicted && s2.classId == BallTracker.CLASS_RED,
                "blue within 12 deg does not steal the lock (still real red)");

        // red now occluded, blue alone: still the (RED) lock coasting on predicted angles
        src.dets = detsOf(new BallTracker.RawDet(BallTracker.CLASS_BLUE, 6.0, -6.0, 0.9, 0.3));
        BallTracker.Sighting s3 = t.update();
        check(s3 != null && s3.classId == BallTracker.CLASS_RED && s3.predicted,
                "red occluded -> coast on predicted red, not adopt blue");

        // beyond LOCK_LOST_MS the lock drops; only then is blue fair game
        try { Thread.sleep(310); } catch (InterruptedException e) { return; }
        BallTracker.Sighting drop = t.update();
        check(drop == null, "red gone past LOCK_LOST_MS -> lock dropped (no target)");
        src.dets = detsOf(new BallTracker.RawDet(BallTracker.CLASS_BLUE, 6.0, -6.0, 0.9, 0.3));
        BallTracker.Sighting s4 = t.update();
        check(s4 != null && s4.classId == BallTracker.CLASS_BLUE, "after the drop, blue is fair game");
    }

    private static void testTrackerScanGating() {
        System.out.println("tracker: scan-time gating (staleness + class filter)");
        ScriptedSource src = new ScriptedSource();
        BallTracker t = new BallTracker(src);
        t.setAllowedClasses(BallTracker.CLASS_RED);

        src.dets = detsOf(new BallTracker.RawDet(BallTracker.CLASS_RED, -1, -7, 0.9, 0.3),
                          new BallTracker.RawDet(BallTracker.CLASS_BLUE, 6, -6, 0.9, 0.3));
        src.staleness = 40;
        List<BallTracker.RawDet> conf = t.getConfidentResults();
        check(conf != null && conf.size() == 2, "getConfidentResults sees every class");
        List<BallTracker.RawDet> gated = t.getGatedDetections();
        check(gated != null && gated.size() == 1 && gated.get(0).classId == BallTracker.CLASS_RED,
                "getGatedDetections honors the allowed-class filter");

        t.reset();
        src.staleness = BallTracker.MAX_STALENESS_MS + 1;
        check(t.getConfidentResults() == null && t.getGatedDetections() == null,
                "both scan helpers go null past MAX_STALENESS_MS");
        check(t.update() == null, "stale frame with no lock yields nothing");
    }
}