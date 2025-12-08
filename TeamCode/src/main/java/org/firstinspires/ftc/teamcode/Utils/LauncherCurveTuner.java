package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.MainBot;

import java.io.File;
import java.util.ArrayList;

@Config
@TeleOp
public class LauncherCurveTuner extends LinearOpMode {
    /*
    Runs launcher at various powers, then writes
    resulting RPM to a file for further processing
    to create a curve of rpm vs. power for more
    accurate launcher control
     */

    public static String TEST_POWERS = "0.1,0.2,0.3";
    public static double STABILIZE_TIME = 10; // seconds
    public static String WRITE_PATH = "";

    @Override
    public void runOpMode() throws InterruptedException {
        MainBot bot = MainBot.shared = new MainBot(hardwareMap, telemetry);

        ArrayList<Double> testPowers = new ArrayList<>();
        for (String strPower: TEST_POWERS.split(",")) {
            testPowers.add(Double.parseDouble(strPower));
        }

        bot.telemetry.addLine("Will test "+testPowers.size()+" powers");
        bot.telemetry.update();

        ArrayList<String> data = new ArrayList<>();

        for (double power: testPowers) {
            bot.launcher.spinUp(power);
            wait((int)(STABILIZE_TIME*1000));
            double rpm = bot.launcher.motor.getRPM();
            data.add(power+","+rpm);
            bot.telemetry.addLine(power+","+rpm);
            bot.telemetry.update();
        }

        bot.telemetry.addLine("Test Results");
        String fileData = "";
        for (String line: data) {
            bot.telemetry.addLine(line);
            fileData += line + "\n";
        }

        if (WRITE_PATH.isBlank()) {
            File file = new File(WRITE_PATH);
            ReadWriteFile.writeFile(file, fileData);
            bot.telemetry.addLine("Wrote data to '" + WRITE_PATH + "'");
        }

        bot.telemetry.update();
    }
}