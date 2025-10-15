package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.SBAs.SBARunner;

@TeleOp(name = "AimBot Tester", group = "Testing")
public class AimBotTester extends OpMode {
    public MainBot bot;
    public SBARunner runner;
    public AimBot aimBot;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;

        aimBot = new AimBot();

        runner = new SBARunner();
        runner.runSBAs(aimBot.getAimSBA());
    }

    @Override
    public void loop() {
        runner.loop();
        if (runner.curSBAs.size() < 1) {
            // Found goal april tag within tolerance, re run for testing
            runner.runSBAs(aimBot.getAimSBA());
        }
    }

    @Override
    public void stop() {
        bot.stop();
    }
}
