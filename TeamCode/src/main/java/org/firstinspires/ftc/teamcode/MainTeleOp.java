package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.SBAs.SBA;

@Config
@TeleOp
public class MainTeleOp extends OpMode {
    public MainBot bot;

    public static double DRIVE_FACTOR = 0.3;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            bot.launcher.kickUp();
        } else {
            bot.launcher.kickDown();
        }
//
        if (gamepad1.a) {
            bot.intake.spinUp();
        } else if (gamepad1.b) {
            bot.intake.doStop();
        }

        if (gamepad1.x) {
            bot.launcher.spinUp();
        } else if (gamepad1.y) {
            bot.launcher.doStop();
        }
//
        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();
        telemetry.update();

        bot.moveBotMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, DRIVE_FACTOR);
    }
}
