// BallMath.java - pure math mirror of math_core.py (inches / degrees).
// No external dependencies: drop into TeamCode/src/org/firstinspires/ftc/teamcode/
// (and run main() with a JDK to dry-run the same self-test as math_core.py).
package org.firstinspires.ftc.teamcode;

public final class BallMath {
    private BallMath() {
    }

    // Unit vector toward the ball in the robot frame (x=forward, y=left, z=up).
    // pinhole/tangent convention, matching the Limelight's tx/ty.
    public static double[] camRay(double txDeg, double tyDeg, double pitchDeg) {
        double tx = Math.toRadians(txDeg);
        double ty = Math.toRadians(tyDeg);
        double p = Math.toRadians(pitchDeg);
        double txn = Math.tan(tx);
        double tyn = Math.tan(ty);
        double zc = 1.0 / Math.sqrt(1.0 + txn * txn + tyn * tyn);
        double xc = txn * zc;
        double yc = tyn * zc;
        double cf = Math.cos(p);
        double sf = Math.sin(p);
        return new double[]{zc * cf + yc * sf, -xc, -zc * sf + yc * cf};
    }

    // Field (x, y) of the ball center + distance + bearing to it.
    // Returns null if the ray never reaches the ball's height.
    public static double[] ballFieldPos(double rx, double ry, double headingDeg,
                                 double txDeg, double tyDeg, double pitchDeg,
                                 double camH, double ballH) {
        double h = Math.toRadians(headingDeg);
        double[] v = camRay(txDeg, tyDeg, pitchDeg);
        double wx = v[0] * Math.cos(h) - v[1] * Math.sin(h);
        double wy = v[0] * Math.sin(h) + v[1] * Math.cos(h);
        if (v[2] >= -1e-9) {
            return null;
        }
        double t = (ballH - camH) / v[2];
        double bx = rx + t * wx;
        double by = ry + t * wy;
        return new double[]{bx, by, Math.hypot(bx - rx, by - ry),
                Math.toDegrees(Math.atan2(by - ry, bx - rx))};
    }

    // Distance from apparent angular size: dist = sqrt(K / ta). ta in 0..1 fraction.
    public static Double rangeFromTa(double taFrac, double calibK) {
        if (taFrac <= 1e-9) {
            return null;
        }
        return Math.sqrt(calibK / taFrac);
    }

    // Robot-center pose so the intake mouth lands on the ball.
    public static double[] pickupPose(double bx, double by, double headingToDeg, double intakeReach) {
        double h = Math.toRadians(headingToDeg);
        return new double[]{bx - intakeReach * Math.cos(h), by - intakeReach * Math.sin(h), headingToDeg};
    }

    // ---- self test mirrors math_core.py ----
    private static double[] invertAngles(double rx, double ry, double headingDeg,
                                         double pitchDeg, double camH, double ballH,
                                         double bx, double by) {
        double h = Math.toRadians(headingDeg);
        double p = Math.toRadians(pitchDeg);
        double dbx = bx - rx;
        double dby = by - ry;
        double lx = dbx * Math.cos(h) + dby * Math.sin(h);
        double ly = -dbx * Math.sin(h) + dby * Math.cos(h);
        double z = ballH - camH;
        double cf = Math.cos(p);
        double sf = Math.sin(p);
        double fc = lx * cf - z * sf;
        double uc = lx * sf + z * cf;
        return new double[]{Math.toDegrees(Math.atan(-ly / fc)), Math.toDegrees(Math.atan(uc / fc))};
    }

    public static void main(String[] args) {
        double camH = 12.0, ballH = 3.0, pitch = 25.0;
        double[][] cases = {
                {0, 0, 0, 80, 0}, {0, 0, 0, 60, 30}, {0, 0, 0, 120, -55},
                {10, -20, 40, 95, 60}, {0, 0, -90, 40, -10}};
        for (double[] c : cases) {
            double[] a = invertAngles(c[0], c[1], c[2], pitch, camH, ballH, c[3], c[4]);
            double[] o = ballFieldPos(c[0], c[1], c[2], a[0], a[1], pitch, camH, ballH);
            if (o == null || Math.abs(o[0] - c[3]) > 1e-6 || Math.abs(o[1] - c[4]) > 1e-6) {
                throw new AssertionError("round trip failed");
            }
        }
        double[] a = invertAngles(0, 0, 10, pitch, camH, ballH, 84, 36);
        double[] b = ballFieldPos(0, 0, 10, a[0], a[1], pitch, camH, ballH);
        double[] pk = pickupPose(b[0], b[1], b[3], 12.0);
        if (Math.abs(Math.hypot(b[0] - pk[0], b[1] - pk[1]) - 12.0) > 1e-6) {
            throw new AssertionError("pickup offset failed");
        }
        System.out.println("BallMath self-test OK: 5 round trips + pickup offset");

        a = invertAngles(0, 0, 20, pitch, camH, ballH, 84, 36);
        b = ballFieldPos(0, 0, 20, a[0], a[1], pitch, camH, ballH);
        pk = pickupPose(b[0], b[1], b[3], 12.0);
        System.out.printf("camera sees tx=%+.2f deg ty=%+.2f deg%n", a[0], a[1]);
        System.out.printf("ball at x=%.2f y=%.2f dist=%.2f bearing=%+.2f%n", b[0], b[1], b[2], b[3]);
        System.out.printf("robot pose x=%.2f y=%.2f heading=%+.2f%n", pk[0], pk[1], pk[2]);
    }
}