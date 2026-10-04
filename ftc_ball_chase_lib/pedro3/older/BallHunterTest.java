/*
 * BallHunterTest - TeleOp harness for BallHunter.
 *
 *   Right bumper (HOLD) : run the hunt. Releasing it aborts instantly and returns manual control.
 *   Left stick / right stick X : normal robot-centric driving.   Left bumper : intake.
 *
 * Set STARTING_POSE to where the robot really is on the field before you press start,
 * otherwise the field-position estimates (and the FIELD_MIN/MAX clamps) will be off.
 */
package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Ball Hunter Test", group = "Hive Vision")
public class BallHunterTest extends LinearOpMode {

    private static final Pose STARTING_POSE = new Pose(72, 72, 0);   // x, y (in), heading (rad)
    private static final int  PIPELINE_INDEX = 0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(STARTING_POSE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();

        DcMotor intake = hardwareMap.tryGet(DcMotor.class, "intake");   // optional

        BallHunter hunter = new BallHunter(follower, limelight, intake);
        hunter.setMaxPickups(3);
        hunter.setTimeBudgetSec(20);

        telemetry.addLine("Hold RIGHT BUMPER to hunt. Release = abort + manual.");
        telemetry.update();
        waitForStart();

        follower.manual(DrivePowers.zero());
        boolean wasHeld = false;

        while (opModeIsActive()) {
            follower.update();                       // once per loop, always

            boolean held = gamepad1.right_bumper;
            if (held && !wasHeld) hunter.start();
            if (!held && wasHeld) hunter.abort();    // instant hand-back to manual
            wasHeld = held;

            if (held) {
                hunter.update();
            } else {
                DrivePowers robotCentric = new DrivePowers(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                follower.manual(ManualDrive.fieldCentric(robotCentric, follower.pose().heading()));
                if (intake != null) intake.setPower(gamepad1.left_bumper ? 1.0 : 0.0);
            }

            Pose p = follower.pose();
            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
            hunter.addTelemetry(telemetry);
            telemetry.update();
        }

        hunter.abort();
        limelight.stop();
    }
}