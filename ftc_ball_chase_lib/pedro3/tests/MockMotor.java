package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/** DcMotor stand-in so the self-test can exercise the drive logic on the robot
 *  with nothing (and no field) attached. Copy of ftc_ball_chase_lib/tests/MockMotor.java. */
public class MockMotor implements DcMotor {

    public double power = 0;
    public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.BRAKE;
    public int setPowerCalls = 0;
    public int directionCalls = 0;
    public int targetPosition = 0;
    public boolean powerFloat = false;
    /** Largest |power| ever commanded, keeping its sign. Several verbs (intakeOn,
     *  reverse) are single-frame steps that zero themselves the moment they
     *  finish, so sampling power after the run always reads 0. */
    public double peakPower = 0;

    public void reset() {
        power = 0;
        setPowerCalls = 0;
        directionCalls = 0;
        peakPower = 0;
    }

    @Override public void setPower(double p) {
        power = p;
        setPowerCalls++;
        if (Math.abs(p) > Math.abs(peakPower)) peakPower = p;
    }
    @Override public double getPower() { return power; }

    @Override public void setDirection(DcMotorSimple.Direction d) { direction = d; directionCalls++; }
    @Override public DcMotorSimple.Direction getDirection() { return direction; }

    @Override public MotorConfigurationType getMotorType() { return null; }
    @Override public void setMotorType(MotorConfigurationType t) { }

    @Override public DcMotorController getController() { return null; }
    @Override public int getPortNumber() { return 0; }

    @Override public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior z) { zeroPower = z; }
    @Override public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() { return zeroPower; }

    @Override public void setPowerFloat() { powerFloat = true; power = 0; }
    @Override public boolean getPowerFloat() { return powerFloat; }

    @Override public void setTargetPosition(int p) { targetPosition = p; }
    @Override public int getTargetPosition() { return targetPosition; }

    @Override public boolean isBusy() { return false; }
    @Override public int getCurrentPosition() { return targetPosition; }

    @Override public void setMode(DcMotor.RunMode m) { mode = m; }
    @Override public DcMotor.RunMode getMode() { return mode; }

    @Override public HardwareDevice.Manufacturer getManufacturer() { return null; }
    @Override public String getDeviceName() { return "MockMotor"; }
    @Override public String getConnectionInfo() { return "mock"; }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() { }
    @Override public void close() { }
}