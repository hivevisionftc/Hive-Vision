/*
 * RobotPose - drivetrain-neutral field pose, inches + radians (heading CCW+
 * from the +X "red wall" axis). The Pedro variant maps to and from
 * com.pedropathing.math.Pose; the mecanum variant takes the same shape
 * from a plug-in PoseRouter.
 */
package org.firstinspires.ftc.teamcode.wrapper;

public final class RobotPose {
    public final double x, y;
    public final double headingRad;

    public RobotPose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    public RobotPose inDeg(double headingDeg) {
        return new RobotPose(x, y, Math.toRadians(headingDeg));
    }

    @Override public String toString() {
        return String.format("(%.1f, %.1f) h=%+.1f deg", x, y, Math.toDegrees(headingRad));
    }
}