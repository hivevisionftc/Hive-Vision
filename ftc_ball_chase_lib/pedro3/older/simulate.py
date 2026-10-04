#!/usr/bin/env python3
"""Headless closed-loop test of the Limelight 3A pickup example.

No robot, no FTC SDK, no pathing library. Everything the OpMode's decision logic
needs is exercised in pure Python:

  1. SYNTHESIZE the Limelight reading: for a known ball position and robot pose,
     invert the camera math to the tx/ty the 3A "would" report, and synthesize
     ta from a calibrated K (same K used by range_from_ta).
  2. REPLAY the OpMode loop (driveToBall + goTo from BallPickupOpMode.java):
     MIN_TA gate -> BallMath.ballFieldPos -> pickup_pose -> move/stop decisions.
  3. MOVE a simple kinematic robot (position + heading, capped speeds) toward
     the pickup pose and assert it converges with the intake on the ball.

Because the example's real FTC code is a 1:1 mirror of math_core.py, and this
sim drives the same decisions with the same constants, passing here is strong
evidence the on-robot logic works before you ever power a motor.

Usage:  python simulate.py [--trace] [--scenario N]
"""
import argparse
import math
import math_core

# Mirrors the defaults in BallPickupOpMode.java.
CAM_H = 12.0          # camera optical center height, inches
BALL_H = 3.0          # ball center height, inches
CAM_PITCH = 25.0      # optical axis below horizontal, degrees
INTAKE_REACH = 12.0   # robot center -> intake mouth, inches
MIN_TA = 0.005        # below this ta, "no target"
ARRIVE_DIST = 3.0     # stop when within this many inches of the pose
MAX_CHASE_DIST = 120.0
TA_K = 360.0          # visual: ta = TA_K / d^2  (<=1, >=MIN_TA inside max range)

# Robot kinematics (pure model, not a real drivetrain): simple per-tick
# holonomic mover - position slides straight at the target point, heading
# rotates toward the target heading. Never models a real drivetrain, only
# proves the vision->pose->decision loop converges.
SPEED_PER_TICK = 12.0   # inches of travel per tick toward the target
TURN_PER_TICK = 12.0    # degrees of heading rotation per tick


# ---------------------------------------------------------------- sim robot
class Robot:
    def __init__(self, x, y, heading_deg):
        self.x, self.y, self.heading = x, y, heading_deg

    def pose(self):
        return (self.x, self.y, math.radians(self.heading))

    def step(self, dist_to_target, target_point, target_heading_deg):
        """Move position straight at target_point (capped), rotate heading at target."""
        if dist_to_target > 1e-6:
            step = min(SPEED_PER_TICK, dist_to_target)
            to_deg = math.degrees(math.atan2(
                target_point[1] - self.y, target_point[0] - self.x))
            self.x += step * math.cos(math.radians(to_deg))
            self.y += step * math.sin(math.radians(to_deg))
        # shortest-angle heading turn
        d = (target_heading_deg - self.heading + 180.0) % 360.0 - 180.0
        if abs(d) > 0.5:
            turn = math.copysign(min(TURN_PER_TICK, abs(d)), d)
            self.heading = (self.heading + turn) % 360.0


# ------------------------------------------------------- the OpMode decision
def drive_decision(robot_x, robot_y, heading_deg, tx, ty, ta):
    """Exactly the gating/math flow in BallPickupOpMode.driveToBall().

    Returns ('no target'|'too far to chase'|'ty too low / out of range', None)
            ('mission', pickup_point, target_heading_deg, ball_xy, bearing_deg, dist)
            ('ta fallback', approx_point, approx_heading_deg)
    """
    if ta < MIN_TA:
        return ("no target", None)
    ball = math_core.ball_field_pos(robot_x, robot_y, heading_deg,
                                    tx, ty, CAM_PITCH, CAM_H, BALL_H)
    if ball is None:
        d = math_core.range_from_ta(ta, TA_K)
        if d is None or d > MAX_CHASE_DIST:
            return ("ty too low / out of range", None)
        approx_bearing = heading_deg + tx
        pt = (robot_x + d * math.sin(math.radians(approx_bearing)),
              robot_y + d * math.cos(math.radians(approx_bearing)))
        return ("ta fallback", pt, approx_bearing)
    bx, by, dist, bearing = ball
    if dist > MAX_CHASE_DIST:
        return ("too far to chase", None)
    px, py, ph = math_core.pickup_pose(bx, by, bearing, INTAKE_REACH)
    return ("mission", (px, py), ph, (bx, by), bearing, dist)


# ------------------------------------------------------------------- setup
SCENARIOS = [
    # (label, robot x, robot y, robot heading, ball x, ball y, expected status)
    ("forward ball, offset heading", 0.0, 0.0, 20.0, 84.0, 36.0, "pickup READY"),
    ("diagonal + far", 40.0, -30.0, -40.0, 112.0, 55.0, "pickup READY"),
    ("facing away from ball", 0.0, 0.0, 180.0, -20.0, 10.0, "pickup READY"),
    ("tight offset near wall", 30.0, 50.0, 90.0, 10.0, 20.0, "pickup READY"),
    ("out-of-range guard declines chase", 0.0, 0.0, 0.0, 200.0, 0.0,
     "too far to chase"),
]


def run_scenario(idx, label, rx, ry, th, bx, by, expected, trace):
    robot = Robot(rx, ry, th)
    ticks = 0
    rows = []
    while ticks < 6000:
        ticks += 1
        # 1. synthesize the 3A reading for the KNOWN ball
        tx, ty = math_core._invert_angles(
            robot.x, robot.y, robot.heading, CAM_PITCH, CAM_H, BALL_H, bx, by)
        dist_to_ball = math.hypot(bx - robot.x, by - robot.y)
        ta = min(1.0, TA_K / (dist_to_ball * dist_to_ball + 1e-9))

        # 2. replay the OpMode decision
        decision = drive_decision(robot.x, robot.y, robot.heading, tx, ty, ta)
        kind = decision[0]
        if kind == "mission":
            _, pt, ph, _ball, _bear, _d = decision
        elif kind == "ta fallback":
            _, pt, ph = decision
        else:
            pt, ph = None, robot.heading

        # 3. move / hold
        if pt is None:
            status = kind
        else:
            err = math.hypot(pt[0] - robot.x, pt[1] - robot.y)
            if err <= ARRIVE_DIST and abs(
                    (ph - robot.heading + 180.0) % 360.0 - 180.0) < 2.0:
                status = "pickup READY"
                robot.step(0.0, pt, ph)
            else:
                status = "driving"
                robot.step(err, pt, ph)

        rows.append((ticks, robot.x, robot.y, robot.heading, tx, ty, ta, status))

        if status in ("pickup READY", "no target", "too far to chase",
                      "ty too low / out of range"):
            break

    last = rows[-1]
    robot_x, robot_y = last[1], last[2]
    status = last[7]
    ok = status == expected
    if kind == "mission":
        px, py, ph = decision[1][0], decision[1][1], decision[2]
        dist_err = math.hypot(px - robot_x, py - robot_y)
        ok = ok and dist_err < 0.5
        print(f"[{'PASS' if ok else 'FAIL'}] {idx}: {label}")
        print(f"    ball({bx:.0f},{by:.0f})  ->  pickup pose "
              f"({px:.1f},{py:.1f}) @ {ph:+.1f}  "
              f"converged in {last[0]} ticks, pos err {dist_err:.2f} in, "
              f"status '{status}'")
    else:
        print(f"[{'PASS' if ok else 'FAIL'}] {idx}: {label}")
        print(f"    detector guard '{status}' fired as expected "
              f"(expected '{expected}') in {last[0]} ticks")
    if trace:
        for r in rows:
            if r[0] % 10 == 0 or r[0] == len(rows):
                print(f"      t={r[0]:>4}  robot=({r[1]:6.1f},{r[2]:6.1f}) "
                      f"h={r[3]:5.1f}  tx={r[4]:+5.1f} ty={r[5]:+5.1f} "
                      f"ta={r[6]:.3f}  {r[7]}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", action="store_true",
                    help="print every 10th tick of each scenario")
    ap.add_argument("--scenario", type=int, default=None,
                    help="run only this scenario index (0-based)")
    args = ap.parse_args()

    all_ok = True
    for i, scen in enumerate(SCENARIOS):
        if args.scenario is not None and i != args.scenario:
            continue
        label, rx, ry, th, bx, by, expected = scen
        all_ok &= run_scenario(f"{i + 1}", label, rx, ry, th, bx, by,
                               expected, args.trace)
    print("ALL SCENARIOS PASS" if all_ok else "SOME SCENARIOS FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())