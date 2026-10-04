/*
 * BallColor - the three ball classes the Latin detector can report, mapped to
 * the teamcode class ids (0 = yellow_pollen, 1 = red_nectar, 2 = blue_nectar).
 * CONFIRM those ids against the Limelight web UI pipeline label list.
 */
package org.firstinspires.ftc.teamcode.wrapper;

import org.firstinspires.ftc.teamcode.BallTracker;

public enum BallColor {
    RED   (BallTracker.CLASS_RED),
    BLUE  (BallTracker.CLASS_BLUE),
    YELLOW(BallTracker.CLASS_YELLOW_NEUTRAL);

    public final int classId;

    BallColor(int classId) {
        this.classId = classId;
    }

    /** Reverse lookup, or null when the id is not one of ours. */
    public static BallColor fromClassId(int classId) {
        for (BallColor c : values()) {
            if (c.classId == classId) return c;
        }
        return null;
    }
}