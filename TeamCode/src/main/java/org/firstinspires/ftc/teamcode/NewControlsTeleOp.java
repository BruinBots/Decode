package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.ObeliskReader;
import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.teamcode.SBAs.SBARunner;

@Config
@TeleOp(name="Correct Controls TeleOp (currently doesn't do anything)", group="Test")
public class NewControlsTeleOp extends OpMode {
    public MainBot bot;
    public SBARunner runner;

    public static double DRIVE_FACTOR = 0.6;

    public AimBot aimBot;
    public ObeliskReader obeliskReader;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;

        runner = new SBARunner();
        aimBot = new AimBot();
        obeliskReader = new ObeliskReader();

        bot.launcher.setAimBot(aimBot);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            // reverse intake
        } else if (gamepad1.right_bumper) {
            // intake
        }

        if (gamepad1.y) {
            // launch one
        }

        if (gamepad1.dpad_up) {
            // henryjack up
        } else if (gamepad1.dpad_down) {
            // henryjack down
        }

        if (gamepad1.a) {
            // clear current ball from launcher
        }

        if (gamepad1.x) {
            // aim with apriltags
        }

        if (gamepad1.b) {
            // launch all
        }

        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();

        aimBot.readAprilTag();
        aimBot.doTelemetry();

        telemetry.addData("Obelisk", obeliskReader.read().toString());

        runner.loop();

        telemetry.update();

        bot.moveBotMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, DRIVE_FACTOR);
    }
}
