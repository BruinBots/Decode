package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends OpMode {

    public DcMotorEx testMotor;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "test");
    }

    @Override
    public void loop() {
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setPower(1);

        telemetry.addData("speed", testMotor.getVelocity(AngleUnit.DEGREES)/360.0*60.0);
    }
}
