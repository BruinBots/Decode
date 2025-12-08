package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import java.io.File;

@Config
@TeleOp(name = "FileUtilityTester", group = "Test")
public class FileUtilityTester extends OpMode {

    // Dashboard-configurable fields
    public static String directoryPath = "/sdcard/FIRST/";   // directory to list
    public static String writePath = "/sdcard/FIRST/test.txt"; // file to write
    public static String writeData = "Hello from BruinBots!";  // data to write
    public static String readPath = "/sdcard/FIRST/test.txt";  // file to read

    private FtcDashboard dashboard;
    private FileDownloadServer server;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        try {
            server = new FileDownloadServer(8082);
            telemetry.addLine("Server running on port 8082");
        } catch (Exception e) {
            telemetry.addLine("Server FAILED: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        Telemetry dashboardTele = dashboard.getTelemetry();

        // -----------------------------
        // A: LIST DIRECTORY
        // -----------------------------
        if (gamepad1.a) {
            File dir = new File(directoryPath);

            if (dir.exists() && dir.isDirectory()) {
                File[] files = dir.listFiles();

                telemetry.addLine("Directory Listing:");
                dashboardTele.addLine("Directory Listing:");

                if (files != null) {
                    for (File f : files) {
                        String name = (f.isDirectory() ? "[D] " : "[F] ") + f.getName();
                        telemetry.addLine(name);
                        dashboardTele.addLine(name);
                    }
                } else {
                    telemetry.addLine("(empty)");
                    dashboardTele.addLine("(empty)");
                }
            } else {
                telemetry.addLine("Invalid directory");
                dashboardTele.addLine("Invalid directory");
            }
        }

        // -----------------------------
        // B: WRITE TO FILE
        // -----------------------------
        if (gamepad1.b) {
            File file = new File(writePath);

            try {
                ReadWriteFile.writeFile(file, writeData);
                telemetry.addLine("Wrote to file!");
                dashboardTele.addLine("Wrote to file!");
            } catch (Exception e) {
                telemetry.addLine("Write ERROR: " + e.getMessage());
                dashboardTele.addLine("Write ERROR: " + e.getMessage());
            }
        }

        // -----------------------------
        // X: READ FILE
        // -----------------------------
        if (gamepad1.x) {
            File file = new File(readPath);

            try {
                String data = ReadWriteFile.readFile(file);
                telemetry.addLine("Read File:");
                telemetry.addLine(data);

                dashboardTele.addLine("Read File:");
                dashboardTele.addLine(data);
            } catch (Exception e) {
                telemetry.addLine("Read ERROR: " + e.getMessage());
                dashboardTele.addLine("Read ERROR: " + e.getMessage());
            }
        }

        telemetry.update();
        dashboardTele.update();
    }
}