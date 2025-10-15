package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AimBot {
    private AprilTags aprilTags;

    public boolean foundGoal = false;
    public double distance = 0.0;
    public double horizontal = 0.0;
    public double rawAngleError = 180.0;
    public double angleError = 180.0;

    // Launcher power constants
    public static double MIN_DISTANCE = 37.0;

    public static double CLOSE_POWER = 0.65;
    public static double CLOSE_DISTANCE = 51.0;

    public static double FAR_POWER = 0.80;
    public static double FAR_DISTANCE = 82.0;

    // Turn bot P controller constants
    public static double TURN_kP = 0.05;
    public static double TURN_MIN_POWER = 0.15;
    public static double TURN_MAX_POWER = 0.35;

    // More turning constants
    public static double ADJUST_kP = 0.04;

    public AimBot() {
        aprilTags = MainBot.shared.aprilTags;
    }

    public void readAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTags.readAprilTags();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                distance = detection.ftcPose.y;

                // compute raw angle
                horizontal = detection.ftcPose.x;
                rawAngleError = -Math.toDegrees(Math.atan(horizontal / distance)); // detection.ftcPose.yaw;

                // Angle-adjusted aimng algorithm
                // https://discord.com/channels/775043247802286080/775048219465220128/1427015590161420450
                double yaw = detection.ftcPose.yaw;
                double angleAdjust = -yaw*ADJUST_kP;
                angleError = rawAngleError - angleAdjust;

                foundGoal = true;
            }
        }
    }

    public double getLaunchPower() {
        if (distance < MIN_DISTANCE || distance > FAR_DISTANCE) {
            return 0.0;
        } else if (distance < CLOSE_DISTANCE) {
            return CLOSE_POWER;
        } else {
            double slope = (FAR_POWER - CLOSE_POWER) / (FAR_DISTANCE - CLOSE_DISTANCE);
            return CLOSE_POWER + slope * (distance - CLOSE_DISTANCE);
        }
    }

    public double getTurnPower() {
        if (Math.abs(angleError) > 30.0) {
            return 0.0;
        }
        if (Math.abs(angleError) < 5.0) {
            return 0.0;
        }
        double power = -TURN_kP * angleError;
        if (Math.abs(power) < TURN_MIN_POWER) {
            return Math.copySign(TURN_MIN_POWER, power);
        } else if (Math.abs(power) > TURN_MAX_POWER) {
            return Math.copySign(TURN_MAX_POWER, power);
        } else {
            return power;
        }
    }

    public void doTelemetry() {
        if (foundGoal) {
            MainBot.shared.telemetry.addData("AimBot Read", distance + "ft, " + angleError + "º");
            MainBot.shared.telemetry.addData("AimBot Result", getLaunchPower() + "L, " + getTurnPower()+"T");
        } else {
            MainBot.shared.telemetry.addData("AimBot Read", "N/A");
            MainBot.shared.telemetry.addData("AimBot Result", "N/A");
        }
    }

    public class AimBotSBA implements SBA {
        private AimBot aimBot;

        public AimBotSBA(AimBot aimBot) {
            this.aimBot = aimBot;
        }

        public void preInit() {
            aimBot.aprilTags.start();
        }

        public boolean sanity() {
            aimBot.readAprilTag();
            return foundGoal && distance >= 10.0 && distance <= 100.0 && Math.abs(angleError) <= 30.0;
        }

        public void init() { }

        public void loop() {
            readAprilTag();
            double launchPower = getLaunchPower();
            double turnPower = getTurnPower();
            if (foundGoal) {
                MainBot.shared.telemetry.addData("Goal Distance", distance);
                MainBot.shared.telemetry.addData("Goal Horizontal Distance", horizontal);
                MainBot.shared.telemetry.addData("Goal RAW Angle Error", rawAngleError);
                MainBot.shared.telemetry.addData("Goal Angle Error", angleError);
            } else {
                MainBot.shared.telemetry.addData("Goal Distance", "N/A");
                MainBot.shared.telemetry.addData("Goal Horizontal Distance", "N/A");
                MainBot.shared.telemetry.addData("Goal RAW Angle Error", "N/A");
                MainBot.shared.telemetry.addData("Goal Angle Error", "N/A");
            }
            MainBot.shared.telemetry.addData("Launch Power", launchPower);
            MainBot.shared.telemetry.addData("Turn Power", turnPower);
            MainBot.shared.telemetry.update();

            MainBot.shared.moveBotMecanum(0, turnPower, 0, 1);
        }

        public boolean isBusy() {
            return !foundGoal || Math.abs(angleError) >= 2.0;
        }
    }

    public AimBotSBA getAimSBA() {
        return new AimBotSBA(this);
    }
}
