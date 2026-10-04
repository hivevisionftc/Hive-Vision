# Limelight 3A ball chase — BallTracker + drivers

Shared perception + drive logic to make a robot pick up BioBuzz's game
elements (`yellow_pollen` / `red_nectar` / `blue_nectar`) using a Limelight 3A SSD
neural detector. Everything is camera-relative or field-coordinate; no extra
helper download needed (the FTC SDK ships `Limelight3A` natively).

## Compatibility (what this library is verified against)

| Component | Tested version | Notes |
|-----------|----------------|-------|
| FTC SDK (`RobotCore` / `Hardware`) | **11.2.1** | 11.x expected; compile-checked against 11.2.1 |
| Pedro Pathing (`core` / `ftc`) | **2.1.2** | needed only for `BallChaseFollower` / `BallHunt` / the wrapper's `PedroWrangler` |
| Limelight | **3A** (SSD neural on CPU) | [`best_limelight3a_ssd_mobilenetv2_300x300.tflite`](../neural-net/weights/) |
| Java / Android | FTC's bundled toolchain | Java 17 toolchain compiled by this repo's `compile_check.cmd` |

**If you use different versions, compatibility is not guaranteed.** The
compile-check and all self-tests run against exactly the versions above.
Tuning in a new SDK year on Pedro 3.x or different Limelight firmware may
require touching `HiveConfig` only — the API surface is stable.

### Pedro 3 (beta)

[`pedro3/`](pedro3/) is a self-contained **beta port to Pedro Pathing 3.x +
FTC SDK 12**, for testers who want to try the new pathing stack. It is a
parallel copy — the 2.1.2 code in this folder is unchanged and still the
supported path. Pick one folder; don't compile both (same package/class names).

| Component | Tested version |
|-----------|----------------|
| FTC SDK (`RobotCore` / `Hardware`) | **12.0.0** |
| Pedro Pathing (`revhub`) | **3.0.1** |

Caveats: `pedro3/` was ported from the standalone example set, so it predates
`HiveConfig`/`PickupConfirmer` and its own `compile_check.cmd` does not run the
repo's `tests/` suite. All hardware constants in `pedro3/older/pedroPathing/Constants.java`
are placeholders, and Pedro 3 changed the field coordinate convention — audit
your poses before running on a field. Not recommended for competition use yet.

New here? Start with the [5-minute quick start](../docs/quick_start.md) and the
[calibration + coordinate conventions](../docs/calibration.md).

## Layout

- **`final/`** — the ship-ready library. `BallTracker` (perception), the two
  chase drivers, and `BallHunt`; copy them into your
  `org.firstinspires.ftc.teamcode` package.
- **`wrapper/`** — the fluent, drivetrain-agnostic verb API (`BallWrangler`
  + `PedroWrangler` / `MecanumWrangler`); see [MODULES.md](MODULES.md).
- **[MODULES.md](MODULES.md)** — full class-level reference (state machines,
  lock semantics, verb API) that used to live in the Java file banners.

## Using it (teleop)

```java
BallTracker tracker = new BallTracker(limelight);
tracker.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);   // or CLASSES_ALL

BallChaseController chase = new BallChaseController(tracker, lf, rf, lb, rb, intake);
chase.setMaxPickups(3);
chase.start();
while (opModeIsActive() && !chase.isDone()) {
    chase.update();
    chase.addTelemetry(telemetry);
}
chase.abort();
```

## Using it (auto, with Pedro)

```java
// YOU own follower.update() - this class never calls it
BallChaseFollower hunt = new BallChaseFollower(follower, tracker, intake);
hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
hunt.setMaxPickups(3); hunt.setTimeBudgetSec(20);
hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();
    hunt.update();
    hunt.addTelemetry(telemetry);
}
hunt.abort();
```

## Using it (auto, the one-liner)

For an autonomous that just wants a few balls, `BallHunt` wraps the same
state machine so the whole hunt is one line — it owns `follower.update()`,
streams telemetry, and aborts cleanly:

```java
@Autonomous
class MyAuto extends LinearOpMode {
    @Override public void runOpMode() {
        Follower follower = BallChaseFollower.buildFollower(this);   // YOUR team's follower
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // "get 2 red balls, 20 seconds max" — drives the whole approach:
        int picked = new BallHunt(follower, limelight, intake)
                .reds().collect(2).within(20)
                .go(this);

        // ...drive to the backdrop and score them...
    }
}
```

Fluent options: `.reds()` `.blues()` `.yellows()` `.alliance()` (default)
`.everything()`  ·  `.collect(n)` balls (≤0 = unlimited)  ·  `.within(sec)`.
Loop-driven users can instead `start()` / `update()` / `abort()` / `isDone()`.

## Calibrate before running

All of these live in **one file** — `final/HiveConfig.java`. The literal
measurement procedure for each one plus the coordinate sign conventions is in
[../docs/calibration.md](../docs/calibration.md).

| Constant | Where | Meaning |
|----------|-------|---------|
| `CAM_PITCH_DEG`, `CAM_H`, `BALL_H` | `final/HiveConfig.java` | Camera tilt below horizontal, lens height, ball-center height (inches). Read at point of use by BallTracker / BallChaseFollower / BallWrangler projection. |
| `CAM_X_OFFSET`, `CAM_Y_OFFSET` | `final/HiveConfig.java` | Camera mount offset from robot center (forward +X, LEFT +Y, inches). Applied when projecting field coords in `wrapper/BallWrangler.projectBall` and `final/BallChaseFollower` field projection. |
| `MIN_CONF`, `MAX_STALENESS_MS` | `final/HiveConfig.java` | Detection gating. `MAX_STALENESS_MS=150`; the SDK reports staleness in MILLISECONDS. The 3A firmware reports confidence 0-1 — `MIN_CONF=0.44` is that scale (the score formula normalizes `(conf - MIN_CONF) / (1 - MIN_CONF)`). |
| Class ids 0/1/2 | `final/HiveConfig.java` | CONFIRM against the Limelight web UI pipeline label list — nothing checks the model at runtime. |
| Drive/turn/coast gains, search/pickup timing | `final/HiveConfig.java` | Everything the drivers read at point of use — one place to retune. |
| `HFOV_DEG`/`VFOV_DEG`, pod/offset geometry | `final/HiveConfig.java` | Camera FOV and mount offsets for field projection. |

## Compile check (structure verification)

The whole tree (`final/` + `wrapper/` + `tests/`) is verified to compile with
`javac` against the official jars (FTC SDK 11.2.1 + Pedro Pathing 2.1.2) plus
JUnit (`@Disabled` bench classes). On this
dev box the jars and a JDK are staged under `%LOCALAPPDATA%\ftc_compile`, so
just run:

```powershell
.\compile_check.cmd -runmath      # compile everything + run the BallMath self-test
.\compile_check.cmd -runwrapper   # compile everything + run the wrapper logic self-test
```

For another machine, pull the Maven Central coordinates into a classpath
(AARs ship a `classes.jar` — extract it onto the classpath):

- `org.firstinspires.ftc:RobotCore:11.2.1`, `org.firstinspires.ftc:Hardware:11.2.1`
- `com.pedropathing:core:2.1.2`, `com.pedropathing:ftc:2.1.2`

```powershell
javac -cp "RobotCore.jar;Hardware.jar;core-2.1.2.jar;ftc.jar;junit-jupiter-api.jar" -d out `
  final\*.java wrapper\*.java tests\*.java
```