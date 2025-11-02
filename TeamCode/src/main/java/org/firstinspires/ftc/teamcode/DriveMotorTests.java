package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveMotorTests extends OpMode {

    private MainBot bot;

    public static double POWER = 0.5;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            bot.leftFrontMotor.setPower(POWER);
        } else if (gamepad1.dpad_up) {
            bot.leftFrontMotor.setPower(-POWER);
        } else {
            bot.leftFrontMotor.setPower(0);
        }

        if (gamepad1.x) {
            bot.leftBackMotor.setPower(POWER);
        } else if (gamepad1.dpad_left) {
            bot.leftBackMotor.setPower(-POWER);
        } else {
            bot.leftBackMotor.setPower(0);
        }

        if (gamepad1.a) {
            bot.rightBackMotor.setPower(POWER);
        } else if (gamepad1.dpad_down) {
            bot.rightBackMotor.setPower(-POWER);
        } else {
            bot.rightBackMotor.setPower(0);
        }

        if (gamepad1.b) {
            bot.rightFrontMotor.setPower(POWER);
        } else if (gamepad1.dpad_right) {
            bot.rightFrontMotor.setPower(-POWER);
        } else {
            bot.rightFrontMotor.setPower(0);
        }
    }
}
