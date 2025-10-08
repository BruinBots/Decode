package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="VelMotorTest", group="Testing")
public class VelMotorTest extends OpMode {
    public DcMotorEx testMotor;
    public CookedMotor cookedMotor;
    public double velocity = 0.0;

    private int TICKS_PER_REVOLUTION = 175; // 28 for launcher
    public static double MAX_CURRENT = 5.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "test");
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cookedMotor = new CookedMotor(testMotor, MAX_CURRENT);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double getVelocity() {
        double ticksPerSecond = testMotor.getVelocity(); // Ticks/sec
        double revsPerSecond = ticksPerSecond / TICKS_PER_REVOLUTION; // RPS
        return revsPerSecond * 60.0; // RPM
    }

    private void setVelocity() {
        double revsPerSecond = velocity / 60.0;
        double ticksPerSecond = revsPerSecond * TICKS_PER_REVOLUTION;
        testMotor.setVelocity(ticksPerSecond);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            velocity += 10.0;
        } else if (gamepad1.dpad_down) {
            velocity -= 10.0;
        } else if (gamepad1.b) {
            velocity = 0;
        }

        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setVelocity();

        telemetry.addData("power", testMotor.getPower());
        telemetry.addData("target speed", velocity);
        telemetry.addData("speed", getVelocity());
        telemetry.addData("position", testMotor.getCurrentPosition());
        telemetry.addData("current", testMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("cooked", cookedMotor.isCooked());
        telemetry.update();

        cookedMotor.loop();
    }
}
