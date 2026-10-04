# Limelight 3A ball pickup / pathing example

Turn a Limelight 3A SSD neural-detector sighting (`yellow_pollen` /
`red_nectar` / `blue_nectar` balls) into a **field-coordinate drive
pose** whose intake mouth lands on the ball. All math lives in two mirror
files — one Python (reference, runs anywhere) and one Java (no-dependency,
drops into FTC TeamCode).

This folder is a self-contained example: copy `BallMath.java` and
`BallPickupOpMode.java` into your `org.firstinspires.ftc.teamcode` package and
tune the constants. The Limelight side needs no helper download — the FTC SDK
ships the `Limelight3A` driver natively.

## Files

| File | What it is |
|------|------------|
| `math_core.py` | Reference math in pure stdlib Python. `python math_core.py` runs a round-trip self-test + demo. |
| `BallMath.java` | Exact Java mirror (`public static` methods, same formulas/units). Compile+run `main()` with a JDK for the same self-test; FTC ignores it. |
| `BallPickupOpMode.java` | FTC `@TeleOp`: native `Limelight3A`/`LLResult` → BallMath → follower pathing. |
| `simulate.py` | Headless closed-loop test: synthesizes 3A readings for known ball positions, replays the OpMode decision logic, drives a kinematic robot to the pickup pose. No robot / no SDK needed. |

## The idea (one ray + a known height)

The 3A already decodes detections. This example reads only the **primary
target**'s `tx` / `ty` / `ta` (the Limelight's per-frame best-ball numbers):

```
LLResult -> tx,ty (deg) + ta (0..1 image fraction)
         -> cam_ray()      unit vector toward the ball in the robot frame
         -> ball_field_pos()  ball (x,y) on the field + distance + bearing
         -> pickup_pose()     robot-CENTER pose so the intake mouth hits the ball
   fallback: range_from_ta()  dist = sqrt(K/ta) when the ray is degenerate
```

Because the ball is a known height above the floor, **one ray is enough**: the
ray from the camera through the ball intersects the horizontal plane at the
ball's center height, and that intersection *is* the ball's field position. No
second camera or distance sensor required. `ball_field_pos` returns `null`
when the ray never drops to that height — the classic sign of a level camera
(see *Tilt* below) — and the code falls back to angular-size range.

### Coordinates / sign conventions (match `math_core.py` and the Limelight)

- **Units**: inches and degrees (FTC convention). The follower wants inches
  too; convert headings to radians where the library expects them.
- **Robot frame**: `x` forward, `y` left, `z` up; heading CCW-positive.
- **`tx`** = horizontal angle from the optical axis (deg, + = target right).
- **`ty`** = vertical angle from the optical axis (deg, + = target **above**
  the image center). Limelight reports exactly this.
- **`pitch`** = optical-axis angle **below** horizontal. The camera must be
  tilted down so a ground ball reads a *positive* `ty` with margin —
  otherwise the ray can never reach the ball's height.
- **`ta`** = 0..1 fraction of the image; `dist = sqrt(K / ta)`.

## Calibrate before running

| Constant | Default | Meaning / how to measure |
|----------|---------|--------------------------|
| `LIMELIGHT_NAME` | `"limelight"` | The 3A's name as shown in the Limelight web UI. |
| `CAM_H` | 12.0 in | Height of the camera optical center above the floor. |
| `BALL_H` | 3.0 in | Height of the ball **center** (in this game the ball radius ≈ 3 in). |
| `CAM_PITCH` | 25.0 deg | Camera tilt below horizontal. Set the camera with a protractor/level. |
| `INTAKE_REACH` | 12.0 in | Robot center → intake-mouth distance along `x` (forward). |
| `MIN_TA` | 0.005 | Below this `ta` treat as "no target" (kills speckle noise). |
| `TA_CALIB_K` | 120.0 | See *Angular-size fallback* below. |
| `MAX_CHASE_DIST` | 120.0 in | Won't drive at a ball "detected" beyond this. |

### Angular-size fallback (`TA_CALIB_K`)

`dist = sqrt(K / ta)`. Calibrate once per camera/mount:

1. Place the ball flat on the floor at a **known** distance `D` (e.g. 60 in),
   facing the camera.
2. Read `ta` from the Limelight web UI / dashboard.
3. `K = D² · ta`. Put that in `TA_CALIB_K`.

This only sets distance; the fallback bearing is approximated as
`heading + tx`. It is a sanity backup — prefer the ray/plane path, which needs
no `K` at all.

## Wiring into an FTC project

1. **Limelight 3A** — the FTC SDK already ships the hardware class, no third-
   party helper to download: `com.qualcomm.hardware.limelightvision.Limelight3A`
   and `com.qualcomm.hardware.limelightvision.LLResult` (see the SDK's own
   `SensorLimelight3A.java` sample). `BallPickupOpMode` gets the 3A from the
   hardware map by `LIMELIGHT_NAME`, calls `start()`, and reads
   `getLatestResult()` every loop.
2. **Pathing library** — add the
   [Pedro-Pathing-Library](https://github.com/Agent-9/Pedro-Pathing-Library)
   (Maven `compile` dependency or copy into `TeamCode`). Configure a
   **drive/turn constants file** and a working **localizer** for your drivetrain
   — `BallPickupOpMode` reads `follower.pose()` every frame, so without a
   localizer the robot does not know where it is.
3. **BallMath + OpMode** — copy `BallMath.java` and `BallPickupOpMode.java`
   into the same package.
4. **On the 3A** — upload `best_limelight3a_ssd_mobilenetv2_300x300.tflite`
   plus `labels.txt`, engine = **CPU**, network-table name matching the
   hardware-map name, and set the confidence slider as tuned (~0.35–0.45).

## Running the self-tests

```powershell
# Python reference (no deps)
python math_core.py
# -> self-test OK: 5 closed-loop cases + pickup-offset heading check
# -> camera sees  tx=-3.37 deg  ty=+19.37 deg
# -> ball at     x=84.00 y=36.00 in  dist=91.39 in  bearing=+23.20 deg
# -> robot pose  robot center x=72.97 y=31.27 heading=+23.20 deg

# Java mirror (needs a JDK; dry-run only, FTC ignores main())
javac -d out org/firstinspires/ftc/teamcode/BallMath.java
java  -cp out  org.firstinspires.ftc.teamcode.BallMath
# -> BallMath self-test OK: 5 round trips + pickup offset
```

The 5 closed-loop cases invert known ball positions back to what the camera
"would see", then re-derive the positions through `BallMath` and require they
round-trip to `< 1e-6`. The pickup case asserts the robot center lands exactly
`INTAKE_REACH` behind the ball and faces it.

## Test the whole loop with no robot at all

`simulate.py` proves the vision → pose → decision logic end to end on a PC —
the only thing it doesn't exercise is the actual motors/sensor. It:

1. **Synthesizes** the `tx`/`ty`/`ta` the 3A *would* report for a known ball
   position (inverting the camera math, `ta = K/d²` with the same `K` used by
   `range_from_ta`).
2. **Replays the exact OpMode decisions** (`MIN_TA` gate → `ball_field_pos` →
   `pickup_pose` → chase/hold), because `BallPickupOpMode` is a 1:1 mirror of
   `math_core.py`.
3. **Drives a kinematic robot** toward the pickup pose and asserts it converges
   with the intake on the ball and the guard rails firing where expected.

```powershell
python simulate.py              # all scenarios, PASS/FAIL summary
python simulate.py --scenario 2 # one scenario (0-based index)
python simulate.py --trace      # per-tick log: robot pose, tx/ty/ta, status
```

Expected: each of the 5 scenarios reports `[PASS]`; the first four converge to
`pickup READY` with ~0 in position error, and the fifth confirms a ball beyond
`MAX_CHASE_DIST` is declined instead of chased.

> The kinematics (`SPEED_PER_TICK`, `TURN_PER_TICK`) are a toy model — they only
> verify the decision loop converges, never real drivetrain behavior. Swap in
> your real localizer/constants and tune the follower on the robot.

## Behavior notes / honest limits

- **Primary target only** — `tx/ty/ta` are the single best detection. The 3A
  emits up to 10 decoded boxes; to collect several balls you must iterate the
  detection list in `LLResult` and select the *next* one, not just `getTX()`.
- **Tilt matters.** With `CAM_PITCH` too small, `ty` stays ≤ 0 for ground
  balls, `ball_field_pos()` returns `null`, and you are stuck on the `ta`
  fallback. Fix the mount, don't patch the math.
- **`ty` sign check** — if your Limelight is mounted lens-up or mirrored in
  the web UI, `ty` inverts and the ray points up; same `null` result.
- **Telemetry is the debugger** — `ball`, `pickup pose`, and `drive` rows show
  exactly which stage is misbehaving.
- The path heading blends from the robot's current heading to the ball
  bearing, so the intake is the leading edge at arrival. `hold()`
  suppresses end-point thrash.