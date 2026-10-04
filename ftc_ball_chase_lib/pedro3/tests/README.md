# Pedro 3 library self-test

`Pedro3LibSelfTest` is a real FTC OpMode **and** a plain desktop program. It
exercises the whole `pedro3/` tree so you can catch logic regressions without
going to the robot.

- OpMode: `Pedro3 Lib Self-Test`, group `Hive Vision`,
  package `org.firstinspires.ftc.teamcode.tests`
- Desktop entry point: `org.firstinspires.ftc.teamcode.tests.Pedro3LibSelfTest`
- Exits non-zero on any failure, so CI / `compile_check.cmd` can gate on it.

## Sections

| Section | Needs | Covers |
| --- | --- | --- |
| **S0** math | nothing | `BallMath`, `Range`, `projectBall`, `groundRange`, camera projection, heading wrap |
| **S1** `BallTracker` | nothing | gating by colour/confidence/size, per-frame selection, target lock, `LOCK_GATE_DEG` snap, predicted hold, `LOCK_LOST_MS` timeout, staleness, `getConfidentResults`, `getGatedDetections`, `reset` |
| **S2** `BallChaseController` | nothing | state machine `SEARCHING`/`COASTING`/`CHASING`/`PICKUP`/`DONE`, mecanum mixdown signs, `MAX_FWD`/`MAX_TURN` saturation, `AIM_TOL_DEG`, `DRIVE_MIN_TX` turn-in-place, intake gating, pickup dwell, `maxPickups` cap, `abort`, `start` re-arm |
| **S3** `BallWrangler` | nothing | every verb (`forward`/`strafe`/`turn`/`gotoPose`/`scan`/`stop`/`reverse`/`shoot`/`intakeOn`/`setIntake`), step chaining, pick accounting, colour selectors, per-verb power caps, last-error reporting |
| **S4** hardware | **robot** | Pedro follower init/localization, odometry after a forward burst, mecanum strafe/turn bursts, `BallHunt` (both variants), legacy `BallHunter`, `BallChaseFollower`, `go(LinearOpMode)` end-to-end |

S0–S3 drive a `MockMotor` drivetrain and a scripted `DetectionSource`, so no
hardware, no camera and no Pedro instance are involved.

## Run it without a robot

```powershell
..\compile_check.cmd -runselftest
```

That compiles `final/ older/ wrapper/ tests/` and then runs S0–S3 headlessly.
Expect `RESULT: ALL CHECKS PASSED` and exit code 0. Current baseline on the dev
box: **223 passed, 0 failed, 1 skipped** (the skip is `go(op)` which needs a
`LinearOpMode`).

To run the compiled classes directly:

```powershell
java -cp "out_pedro3;RobotCore-12.0.0.jar;Hardware-12.0.0.jar;pedro3_core.jar;pedro3_revhub.jar" `
  org.firstinspires.ftc.teamcode.tests.Pedro3LibSelfTest
```

## Run S4 on the robot

**Lift the robot or block the wheels first.** S4 drives. S0–S3 are safe with the
robot anywhere.

1. **Edit `older/pedroPathing/Constants.java` for your robot first.** The pod
   offsets, ticks-to-inches, and `MAX_FORWARD_VELOCITY` / `MAX_STRAFE_VELOCITY`
   in there are placeholders. S4 prints a preflight list of every device name it
   needs plus every device your robot actually has, so you can fix the names
   straight from telemetry.
2. Deploy to Team Code, pick `Pedro3 Lib Self-Test` from `Hive Vision`.
3. During INIT, **hold A** to arm. Releasing A or pressing B at any time aborts
   and zeroes the motors.
4. Put the robot on the floor with about a foot of clearance. S4 drives forward,
   backward, strafes, turns, and runs one short 3-inch path.
5. Read the summary at the end. Failures print `FAIL <name>` with the actual
   value.

### Safety interlocks

- Manual bursts are capped at `MOTOR_CAP` (0.30 full scale) for at most
  `BUST_MS` (500 ms).
- **Pedro pathing is *not* power-capped by this test** — it runs at the velocity
  limits in `Constants`. The go-to is therefore gated on an odometry-scale
  interlock: if the measured forward burst is not between 0.05 in and 12 in, the
  path step is *skipped* with an explanation, because running a follower with
  wrong ticks-to-inches is how a robot ends up off the table. The placeholders
  (`0.05` in/tick) are roughly 25x too large on a GoBilda, so this will
  normally skip until you tune them.
- **B is an emergency stop** checked by every S4 loop, not just the motor bursts.
- Every bounded motion zeroes the motors in a `finally` block.
- The whole OpMode is wrapped so that *no* exit path — including an unhandled
  exception — leaves a motor commanded, tunables modified, or the summary
  unwritten.
- S4 is skipped entirely unless A is held, so it is safe to leave the OpMode in
  the menu.

### Known limits

- Section results are printed to telemetry only; nothing is written to a file.
  Screenshot or dump the telemetry if you want a record.
- `PedroWrangler.then(pose)` is exercised for a 3-inch move with a 6-second
  `.within()` tolerance and a 5-second pump cap. It is the only unbounded-power
  step, which is why it sits behind the interlock above.
- The `intake` motor is looked up by the literal name `intake` and is optional;
  if yours is named differently the verb tests still run, just without an
  intake.

## Behaviour this test pins down

These are contracts the code depends on, asserted so a refactor cannot quietly
change them:

- `BallTracker` adopts a new detection inside `LOCK_GATE_DEG`; a larger jump
  holds the *old predicted* lock for `LOCK_LOST_MS`.
- The lock is colour-bound — a different class cannot steal it.
- `BallWrangler` colour selectors gate "any colour" queries and chase steps,
  but `canSee(colour)` / `count(colour)` / `find(colour)` report raw per-class
  visibility. `getGatedDetections()` filters by allowed class and deliberately
  does **not** touch the lock.
- `searchAt(pose)` queues two steps: go-to, then scan.
- `ScanStep` returns immediately if a ball is already visible.
- `IntakeStep` fires a single non-zero frame and then zeroes itself;
  `MockMotor.peakPower` captures the transient.
- `reverse()` reverses the intake, not the drivetrain.
- `drive(fwd, 0, turn)` mixes `pLf = pLb = fwd + turn` and
  `pRf = pRb = fwd - turn`, then normalises by `max(1, maxAbs(...))`. With
  `LF/LB FORWARD` and `RF/RB REVERSE` that makes uniform `+power` drive
  forward and `turn > 0` rotate **clockwise**. Because of the mix, an
  individual motor may exceed `MAX_FWD`; the invariant is `|power| <= 1`.

## Harness notes

The test saves every static tunable it mutates, forces a safe cap, and restores
the originals in a `finally` block. A final check
(`all tunables are restored after the run`) asserts this, so the sections stay
order-independent and the OpMode cannot leave your tuning altered.

`BallChaseController` captures its `BallTracker` at construction. To give it a
clean view mid-run, call `tracker.reset()` — re-assigning a new tracker does not
reach the controller.

`runWranglerPumpStarted` samples `isDone()` *before* calling `abort()`, because
`abort()` forces the done flag. Checking afterwards would pass even if the motion
never ran.