/*
 * BallHunt - the classic one-liner name, now a PedroWrangler: the full verb API
 * without learning a new class name. Fluent coloring/time verbs here return
 * BallHunt (not BallWrangler) so the one-liner stays one line:
 *
 *     int picked = new BallHunt(follower, limelight, intake)
 *             .reds().collect(2).within(20)
 *             .go(this);                    // get 2 red balls, 20 s max
 *
 * or the state machine form:
 *
 *     BallHunt hunt = new BallHunt(follower, limelight, intake);
 *
 *     if (hunt.canSee(RED)) hunt.grab(RED).within(6).go(this);
 *     else                  hunt.scan().then(hunt.grabNearest()).go(this);
 *
 *     hunt.grabTwo(RED, BLUE).thenReturnTo(scorePose).go(this);
 */
package org.firstinspires.ftc.teamcode.wrapper;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.BallTracker;

public class BallHunt extends PedroWrangler {

    public BallHunt(Follower follower, Limelight3A limelight, DcMotor intakeOrNull) {
        super(follower, limelight, intakeOrNull);
    }

    public BallHunt(Follower follower, BallTracker tracker, DcMotor intakeOrNull) {
        super(follower, tracker, intakeOrNull);
    }

    public BallHunt(Follower follower, BallTracker tracker, DcMotor intakeOrNull,
                    FullSensor fullOrNull) {
        super(follower, tracker, intakeOrNull, fullOrNull);
    }

    /* ---------------- fluent returns (keep the one-liner a one-liner) ---------------- */

    @Override public BallHunt reds()      { super.reds(); return this; }
    @Override public BallHunt blues()     { super.blues(); return this; }
    @Override public BallHunt yellows()   { super.yellows(); return this; }
    @Override public BallHunt alliance()  { super.alliance(); return this; }
    @Override public BallHunt everything(){ super.everything(); return this; }
    @Override public BallHunt within(double seconds) { super.within(seconds); return this; }
    @Override public BallHunt clear()     { super.clear(); return this; }
    @Override public BallHunt withPickupSensor(BallWrangler.PickupSensor s) { super.withPickupSensor(s); return this; }

    /** Pick up n balls this hunt (n <= 0 = keep going until none are left). */
    public BallHunt collect(int n) {
        if (n <= 0) grabAll(); else grabUpTo(n);
        return this;
    }

    public String state() {
        return isActive() ? "running" : isDone() ? "done" : "idle";
    }
}