#!/usr/bin/env python3
"""Ball -> drive-pose math for the Limelight 3A SSD (yellow_pollen/nectar) detector.

All units are INCHES and DEGREES (FTC convention). Pure stdlib -> runs anywhere.

Pipeline used by the FTC code:
    LLResult -> tx/ty (degrees)            camera angles to the ball
                ta (0..1 fraction of image) angular size (range fallback)
    cam_ray()                              unit vector ball direction in robot frame
    ball_field_pos()                       ball (x, y) on the field + distance + bearing
    pickup_pose()                          where the robot CENTER must be so the intake
                                           mouth sits on the ball (path target)
    range_from_ta()                        distance from angular size (secondary method)
"""
import math


def cam_ray(tx_deg, ty_deg, pitch_deg):
    """Unit vector from camera toward the ball in the ROBOT frame
    (x = forward, y = left, z = up). pitch_deg = optical axis angle below horizontal.

    Uses the pinhole/tangent convention Limelight reports (tx/ty are the
    horizontal/vertical angle from the optical axis): with t = (tan tx, tan ty),
    the optical-axis component is z = 1/sqrt(1 + tx^2 + ty^2)."""
    txn = math.tan(math.radians(tx_deg))
    tyn = math.tan(math.radians(ty_deg))
    zc = 1.0 / math.sqrt(1.0 + txn * txn + tyn * tyn)
    xc = txn * zc   # camera right
    yc = tyn * zc   # camera up
    cf, sf = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    fwd = zc * cf + yc * sf      # +x forward
    left = -xc                   # +y left  (target to the right -> negative)
    up = -zc * sf + yc * cf      # +z up
    return (fwd, left, up)


def ball_field_pos(rx, ry, heading_deg, tx_deg, ty_deg, pitch_deg, cam_h, ball_h):
    """Field (x, y) of the ball center + ground distance + bearing to it.
    Returns None if the ray never drops to the ball's height."""
    h = math.radians(heading_deg)
    fx, cy, fz = cam_ray(tx_deg, ty_deg, pitch_deg)
    wx = fx * math.cos(h) - cy * math.sin(h)
    wy = fx * math.sin(h) + cy * math.cos(h)
    if fz >= -1e-9:
        return None
    t = (ball_h - cam_h) / fz
    bx, by = rx + t * wx, ry + t * wy
    return (bx, by, math.hypot(bx - rx, by - ry), math.degrees(math.atan2(by - ry, bx - rx)))


def range_from_ta(ta_frac, calib_k):
    """Distance from angular size: dist = sqrt(K / ta). Calibrate K once
    (see README). Returns None when the ball is too small to trust."""
    if ta_frac <= 1e-9:
        return None
    return math.sqrt(calib_k / ta_frac)


def pickup_pose(bx, by, heading_to_deg, intake_reach):
    """Robot-CENTER pose so the intake mouth lands on the ball.
    intake_reach = robot center -> intake mouth distance."""
    h = math.radians(heading_to_deg)
    return (bx - intake_reach * math.cos(h), by - intake_reach * math.sin(h), heading_to_deg)


# ---------------------------------------------------------------- self test
def _invert_angles(rx, ry, heading_deg, pitch_deg, cam_h, ball_h, bx, by):
    """Given a known ball position, what tx/ty would the camera report?
    (used only to build closed-loop test cases)"""
    h = math.radians(heading_deg)
    dbx, dby = bx - rx, by - ry
    lx = dbx * math.cos(h) + dby * math.sin(h)
    ly = -dbx * math.sin(h) + dby * math.cos(h)
    z = ball_h - cam_h
    cf, sf = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    fcomp = lx * cf - z * sf
    ucomp = lx * sf + z * cf
    return math.degrees(math.atan(-ly / fcomp)), math.degrees(math.atan(ucomp / fcomp))


def self_test():
    cam_h, ball_h, pitch = 12.0, 3.0, 25.0
    cases = [
        (0.0, 0.0, 0.0, 80.0, 0.0),
        (0.0, 0.0, 0.0, 60.0, 30.0),
        (0.0, 0.0, 0.0, 120.0, -55.0),
        (10.0, -20.0, 40.0, 95.0, 60.0),
        (0.0, 0.0, -90.0, 40.0, -10.0),
    ]
    for rx, ry, th, bx, by in cases:
        tx, ty = _invert_angles(rx, ry, th, pitch, cam_h, ball_h, bx, by)
        out = ball_field_pos(rx, ry, th, tx, ty, pitch, cam_h, ball_h)
        assert out is not None, (bx, by)
        rx_, ry_, dist, bear = out
        assert math.isclose(rx_, bx, abs_tol=1e-6), (bx, by, rx_, ry_)
        assert math.isclose(ry_, by, abs_tol=1e-6), (bx, by, rx_, ry_)
        assert math.isclose(dist, math.hypot(bx - rx, by - ry), abs_tol=1e-6)

    # pickup pose: robot center offset by intake reach, facing the ball
    bx, by, dist, bear = ball_field_pos(0, 0, 10, * _invert_angles(0, 0, 10, pitch, cam_h, ball_h, 84, 36), pitch, cam_h, ball_h)
    reach = 12.0
    px, py, ph = pickup_pose(bx, by, bear, reach)
    assert math.isclose(math.hypot(bx - px, by - py), reach, abs_tol=1e-6)
    assert math.isclose(abs(((math.atan2(by - py, bx - px) - math.radians(ph)) + math.pi) % (2 * math.pi) - math.pi), 0.0, abs_tol=1e-6)
    print(f"self-test OK: 5 closed-loop cases + pickup-offset heading check")


def demo():
    cam_h, ball_h, pitch = 12.0, 3.0, 25.0
    rx, ry, th = 0.0, 0.0, 20.0
    tx, ty = _invert_angles(rx, ry, th, pitch, cam_h, ball_h, 84.0, 36.0)
    bx, by, dist, bear = ball_field_pos(rx, ry, th, tx, ty, pitch, cam_h, ball_h)
    px, py, ph = pickup_pose(bx, by, bear, 12.0)
    print(f"camera sees  tx={tx:+.2f} deg  ty={ty:+.2f} deg")
    print(f"ball at     x={bx:.2f} y={by:.2f} in  dist={dist:.2f} in  bearing={bear:+.2f} deg")
    print(f"robot pose  robot center x={px:.2f} y={py:.2f} heading={ph:+.2f} deg")


if __name__ == "__main__":
    self_test()
    demo()