package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class AimBot {
    private AprilTags aprilTags;

    public boolean foundGoal = false;
    private long foundGoalTime = 0;
    public double distance = 0.0;
    public double horizontal = 0.0;
    public double rawAngleError = 180.0;
    public double angleError = 180.0;

    public static double MIN_FOUND_TIME = 750; // ms
    public static double ANGLE_TOLERANCE = 3.5;


    public static double TIME_BUFFER = 500; // max time in ms from last found april tag to reading there's no april tag

    // Launcher power constants
//    public static double MIN_DISTANCE = 30.0;
//
//    public static double CLOSE_POWER = 0.70;
//    public static double CLOSE_DISTANCE = 63.0;
//
//    public static double FAR_POWER = 0.80;
//    public static double FAR_DISTANCE = 80.0;
//
//    public static double MAX_DISTANCE = 95.0;

    public static double CLOSE_POWER = 0.74;
    public static double THRESHOLD_DISTANCE = 72.0;
    public static double FAR_POWER = 0.9;

    // Turn bot P controller constants
//    public static double TURN_kP = 0.01;
//    public static double TURN_MIN_POWER = 0.15;
//    public static double TURN_MAX_POWER = 0.45;
    public static double TURN_POWER = 0.15;

    // More turning constants
    public static double ADJUST_kP = 1;
    public static double STATIC_ANGLE_OFFSET = 2;

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
                double angleAdjust = -yaw * ADJUST_kP;
                angleError = rawAngleError - angleAdjust - STATIC_ANGLE_OFFSET;

                foundGoalTime = System.currentTimeMillis();
            }
        }

        foundGoal = (System.currentTimeMillis() - foundGoalTime) < TIME_BUFFER;
    }

    public double getLaunchPower() {
        if (distance < THRESHOLD_DISTANCE) {
            return CLOSE_POWER;
        } else if (distance > THRESHOLD_DISTANCE) {
            return FAR_POWER;
        } else {
            return CLOSE_POWER;
        }
    }

    public double getTurnPower() {
        if (Math.abs(angleError) > 30.0) {
            return 0.0;
        }
        if (Math.abs(angleError) < ANGLE_TOLERANCE) {
            return 0.0;
        }
        return Math.copySign(TURN_POWER, -angleError);
//        if (Math.abs(power) < TURN_MIN_POWER) {
//            return Math.copySign(TURN_MIN_POWER, power);
//        } else
//        if (Math.abs(power) > TURN_MAX_POWER) {
//            return Math.copySign(TURN_MAX_POWER, power);
//        } else {
//            return power;
//        }
    }

    public void doTelemetry() {
        if (foundGoal) {
            MainBot.shared.telemetry.addData("AimBot Read", distance + "in, " + angleError + "º");
            MainBot.shared.telemetry.addData("AimBot Result", getLaunchPower() + "L, " + getTurnPower() + "T");
        } else {
            MainBot.shared.telemetry.addData("AimBot Read", "N/A");
            MainBot.shared.telemetry.addData("AimBot Result", "N/A");
        }
    }

    public class AimBotAction implements Action {

        private AimBot aimBot;
        private boolean firstLoop = true;

        private AimBotAction(AimBot aimBot) {
            this.aimBot = aimBot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                aprilTags.start();

                // Reset found goal variables
                aimBot.foundGoal = false;
                aimBot.foundGoalTime = 0;

                firstLoop = false;
                return true; // Keep iterating later on
            }
            aimBot.readAprilTag();
            double launchPower = getLaunchPower();
            double turnPower = getTurnPower();
//            if (aimBot.foundGoal) {
                MainBot.shared.telemetry.addData("Goal Distance", distance);
                MainBot.shared.telemetry.addData("Goal Horizontal Distance", horizontal);
                MainBot.shared.telemetry.addData("Goal RAW Angle Error", rawAngleError);
                MainBot.shared.telemetry.addData("Goal Angle Error", angleError);
//            } else {
//                MainBot.shared.telemetry.addData("Goal Distance", "N/A");
//                MainBot.shared.telemetry.addData("Goal Horizontal Distance", "N/A");
//                MainBot.shared.telemetry.addData("Goal RAW Angle Error", "N/A");
//                MainBot.shared.telemetry.addData("Goal Angle Error", "N/A");
//            }
            MainBot.shared.telemetry.addData("Launch Power", launchPower);
            MainBot.shared.telemetry.addData("Turn Power", turnPower);
            MainBot.shared.moveBotMecanum(0, turnPower, 0, 1);

            MainBot.shared.telemetry.addData("Time Action", (System.currentTimeMillis() - aimBot.foundGoalTime < AimBot.MIN_FOUND_TIME));
            MainBot.shared.telemetry.addData("Turn Power Condition", turnPower != 0);
            telemetryPacket.addLine("Time Condition " + (System.currentTimeMillis() - aimBot.foundGoalTime < AimBot.MIN_FOUND_TIME));
            telemetryPacket.addLine("Turn Power Condition " + (turnPower != 0));

            return (System.currentTimeMillis() - aimBot.foundGoalTime < AimBot.MIN_FOUND_TIME) || turnPower != 0;
        }
    }

    public AimBotAction getAction() {
        return new AimBotAction(this);
    }
}
