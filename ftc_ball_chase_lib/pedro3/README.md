# Limelight 3A ball chase — BallTracker + drivers

Shared perception + drive logic to make a robot pick up BioBuzz's game
elements (`yellow_pollen` / `red_nectar` / `blue_nectar`) using a Limelight 3A SSD
neural detector. Everything is camera-relative or field-coordinate; no extra
helper download needed (the FTC SDK ships `Limelight3A` natively). Works with
FTC SDK **12.x** and **Pedro Pathing 3.x**.

## Layout

- **`final/`** — the ship-ready code. Copy these four `.java` files straight
  into your `org.firstinspires.ftc.teamcode` package:
- **`wrapper/`** — `MecanumWrangler` / `BallWrangler`, the higher-level verb and
  step-chain API. Not needed if you only use `BallChaseController`.
- **`tests/`** — `Pedro3LibSelfTest`, the self-test OpMode plus its desktop
  runner. Deploy it too; see the Self-test section below.
- **`older/`** — superseded/experimental set (BallMath reference, BallHunter,
  the migrated Pedro 3.x demos). Keeping it out of the active code path; see the
  notes there.

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
        Follower follower = FtcConfig.buildFollower(this);   // YOUR team's follower
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

| Constant | Where | Meaning |
|----------|-------|---------|
| `CAM_PITCH_DEG`, `CAM_H`, `BALL_H` | `final/BallTracker.java` | Camera tilt below horizontal, lens height, ball-center height (inches). |
| `CAM_X_OFFSET`, `CAM_Y_OFFSET` | `final/BallTracker.java` | Camera mount offset from robot center (forward +X, LEFT +Y, inches). Applied when projecting field coords in `wrapper/BallWrangler.projectBall` and `final/BallChaseFollower` field projection. |
| `MIN_CONF`, `MAX_STALENESS_MS` | `final/BallTracker.java` | Detection gating. `MAX_STALENESS_MS=120`; the SDK reports staleness in MILLISECONDS. Check the 3A firmware: confidence 0-1 or 0-100? |
| Class ids 0/1/2 | `final/BallTracker.java` | CONFIRM against the Limelight web UI pipeline label list — nothing checks the model at runtime. |
| Drive/turn/coast gains | `final/BallChaseController.java` / `final/BallChaseFollower.java` | All `public static`, tune in place or via a config system. |
| `HFOV_DEG`/`VFOV_DEG`, pod/offset geometry | `final/BallChaseFollower.java` | Camera FOV and mount offsets for field projection. |

## Self-test

`tests/` holds `Pedro3LibSelfTest`, an OpMode plus desktop program that checks
every public function in this tree against a mock drivetrain and scripted
detections. Sections S0–S3 need no hardware and run headlessly:

```powershell
.\compile_check.cmd -runselftest   # compile everything + run the self-test
```

Section S4 covers Pedro follower initialization, odometry and the OpMode
end-to-end paths; it requires the robot (lift it or block the wheels) and only
runs when A is held during INIT. Before running it, edit
`older/pedroPathing/Constants.java` for your robot — the pod offsets and
ticks-to-inches are placeholders, and S4 refuses to path until a measured burst
proves the odometry scale is sane. See [`tests/README.md`](tests/README.md) for
the full section list, the safety interlocks, and the pinned-down behaviours.

## Compile check (structure verification)

The whole tree (`final/` + `wrapper/` + `older/` + `tests/`) is verified to compile with `javac`
against the official jars (FTC SDK 12.0.0 + Pedro Pathing 3.x). On this
dev box the jars and a JDK are staged under `%LOCALAPPDATA%\ftc_compile`, so
just run:

```powershell
.\compile_check.cmd -runmath   # compile everything + run the BallMath self-test
```

For another machine, build the classpath from these coordinates
(FTC ships AARs — extract the inner `classes.jar`):

- `org.firstinspires.ftc:RobotCore:12.0.0`, `org.firstinspires.ftc:Hardware:12.0.0`
- `com.pedropathing:revhub:3.0.1`, `com.pedropathing:tuning:1.0.1`

```powershell
javac -cp "RobotCore-12.0.0.jar;Hardware-12.0.0.jar;revhub-3.0.1.jar;tuning-1.0.1.jar" -d out `
  final\*.java older\*.java wrapper\*.java tests\*.java older\pedroPathing\Constants.java
```

> **Note on the staged Pedro 3 jars.** At the time of this migration,
> `com.pedropathing:revhub` / `:tuning` were **not yet published** to
> `https://repo.dairy.foundation/releases/` (the documented repo returns 404 for
> `com/pedropathing/...`, and its directory listing only contains
> `com/{acmerobotics,bylazar,qualcomm}`, `dev/frozenmilk`, `org/{...}`).
> The `pedro3_core.jar` / `pedro3_revhub.jar` under `%LOCALAPPDATA%\ftc_compile`
> were therefore **built from the official `v3.0.1` source tag**
> (`github.com/Pedro-Pathing/PedroPathing`, modules `:core` and `:revhub`),
> compiling cleanly against FTC SDK 12.0.0. When the artifacts are published,
> switch the Gradle coordinates above in and drop the local jars.

## `older/`

`older/` holds the superseded example set, moved out and no longer part of the
active code path: the BallMath/math_core/simulate reference (primary-target
only, `ta` fallback), the standalone `BallHunter` / `BallChaseOpMode`, and the
Pedro 1.x demos `BallPickupOpMode` / `BallHunterTest`. Those two were migrated
1.x → 2.1.2 → **3.x** (`Paths.line(...).linear(...)` + `follower.follow(path)`,
`follower.pose()`, `follower.stop()`, `follower.setPose()`,
`follower.hold(Pose, boolean)`, and `Constants.createFollower()` built from
`Configuration`/`ConfigVar` bags) so the archived examples still compile — see
`older/README.md` for their tuning notes. The placeholder
`older/pedroPathing/Constants.java` exists only for those demos; the files in
`final/` don't use it.

> The root-level `BallChaseOpMode.java` is a near-identical duplicate of
> `older/BallChaseOpMode.java` (same fully-qualified class name). `compile_check.cmd`
> compiles only `final/`, `older/` and `wrapper/`, so the root copy is excluded to
> avoid a duplicate-class error.