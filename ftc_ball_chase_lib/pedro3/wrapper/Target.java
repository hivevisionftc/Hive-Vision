/*
 * Target - one detected ball, as the verb API hands it out: everything the
 * robot needs to aim and decide, in inches and degrees, plus an optional
 * projected field position when the drivetrain has localization.
 */
package org.firstinspires.ftc.teamcode.wrapper;

public final class Target {
    public final BallColor color;
    public final double bearingDeg;   // camera tx, + = ball to the RIGHT
    public final double elevDeg;      // camera ty, + = ball UP in image
    public final double distIn;       // ground range camera -> ball, inches
    public final double confidence;   // detector confidence (0..1 on this firmware)
    public final double score;        // lower = better (see BallWrangler.scoreOf)
    public final double fieldX, fieldY;   // NaN until projected from a pose

    public Target(BallColor color, double bearingDeg, double elevDeg,
                  double distIn, double confidence, double score) {
        this(color, bearingDeg, elevDeg, distIn, confidence, score, Double.NaN, Double.NaN);
    }

    private Target(BallColor color, double bearingDeg, double elevDeg,
                   double distIn, double confidence, double score,
                   double fieldX, double fieldY) {
        this.color = color;
        this.bearingDeg = bearingDeg;
        this.elevDeg = elevDeg;
        this.distIn = distIn;
        this.confidence = confidence;
        this.score = score;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
    }

    public Target withField(double x, double y) {
        return new Target(color, bearingDeg, elevDeg, distIn, confidence, score, x, y);
    }

    public boolean hasField() {
        return !Double.isNaN(fieldX);
    }

    @Override public String toString() {
        String f = hasField()
                ? String.format(" field=(%.1f,%.1f)", fieldX, fieldY) : "";
        return String.format("%s  tx=%+.1f ty=%+.1f dist=%.0f in conf=%.2f%s",
                color, bearingDeg, elevDeg, distIn, confidence, f);
    }
}