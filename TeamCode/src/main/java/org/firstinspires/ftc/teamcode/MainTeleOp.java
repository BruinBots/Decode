package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.SBAs.SBA;

@TeleOp
public class MainTeleOp extends OpMode {
    public MainBot bot;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            bot.launcher.spinUp();
        } else if (gamepad1.b) {
            bot.launcher.doStop();
        }

        if (gamepad1.x) {
            bot.intake.spinUp();
        } else if (gamepad1.y) {
            bot.intake.doStop();
        }

        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();
        telemetry.update();
    }
}
