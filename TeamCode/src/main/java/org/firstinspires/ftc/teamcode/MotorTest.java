package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends OpMode {

    public DcMotorEx testMotor;
    public double power = 1.0;

    private boolean lastUp = false;
    private boolean lastDown = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "test");
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && !lastUp) {
            power += 0.05;
        }
        else if (gamepad1.dpad_down && !lastDown) {
            power -= 0.05;
        }
        power = Math.max(0.0, Math.min(1.0, power));

        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setPower(power);

        telemetry.addData("power", power);
        telemetry.addData("speed", testMotor.getVelocity(AngleUnit.DEGREES)/360.0*60.0);
        telemetry.update();

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }
}
