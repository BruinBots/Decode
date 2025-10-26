package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.MainBot;

@Autonomous(name="DoNothing", group="A-Backup")
public class DoNothing extends OpMode {
    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
//        MainBot.shared.init();
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }
}
