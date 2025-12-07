package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.MainBot;

@Autonomous(name="DoNothing")
public class DoNothing extends OpMode {
    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        stop();
    }
}
