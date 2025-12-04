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

//    public static double CLOSE_POWER = 0.71;
    public static double CLOSE_VELOCITY = 3900; // rpm
    public static double THRESHOLD_DISTANCE = 85.0;
//    public static double FAR_POWER = 0.85;
    public static double FAR_VELOCITY = 4600;
    public static double TURN_POWER = 0.11;

    // More turning constants
    public static double ADJUST_kP = 1;
    public static double NEAR_STATIC_OFFSET = 14;
    public static double FAR_STATIC_OFFSET = 28;

    public AimBot() {
        aprilTags = MainBot.shared.aprilTags;
    }

    public void readAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTags.readAprilTags();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                distance = detection.ftcPose.y;

                double staticOffset;
                if (distance > THRESHOLD_DISTANCE) {
                    staticOffset = FAR_STATIC_OFFSET;
                } else {
                    staticOffset = NEAR_STATIC_OFFSET;
                }

                // compute raw angle from trig
                horizontal = detection.ftcPose.x;
                rawAngleError = -Math.toDegrees(Math.atan(horizontal / distance)); // detection.ftcPose.yaw;

                // Angle-adjusted aiming algorithm
                // https://discord.com/channels/775043247802286080/775048219465220128/1427015590161420450
                double yaw = detection.ftcPose.yaw;
                double angleAdjust = -yaw * ADJUST_kP;
                angleError = rawAngleError - angleAdjust - staticOffset;

                foundGoalTime = System.currentTimeMillis();
            }
        }

        foundGoal = (System.currentTimeMillis() - foundGoalTime) < TIME_BUFFER;
        if (!foundGoal) {
            distance = 0;
            angleError = 0;
            rawAngleError = 0;
        }
    }

//    public double getLaunchPower() {
//        if (distance < THRESHOLD_DISTANCE) {
//            return CLOSE_POWER;
//        } else if (distance > THRESHOLD_DISTANCE) {
//            return FAR_POWER;
//        } else {
//            return CLOSE_POWER;
//        }
//    }

    public double getLaunchVel() {
        if (distance < THRESHOLD_DISTANCE) {
            return CLOSE_VELOCITY;
        } else if (distance > THRESHOLD_DISTANCE) {
            return FAR_VELOCITY;
        } else {
            return CLOSE_VELOCITY;
        }
    }

    public double getTurnPower() {
        if (!foundGoal || Math.abs(angleError) > 30.0) {
            return 0;
        } else if (Math.abs(angleError) < ANGLE_TOLERANCE) {
            return 0.0;
        }
        return Math.copySign(TURN_POWER, -angleError);
    }

    public void doTelemetry() {
//        if (foundGoal) {
            MainBot.shared.telemetry.addData("AimBot Read", distance + "in, " + angleError + "º");
            MainBot.shared.telemetry.addData("AimBot Result", getLaunchVel() + "L, " + getTurnPower() + "T");
//        } else {
//            MainBot.shared.telemetry.addData("AimBot Read", "N/A");
//            MainBot.shared.telemetry.addData("AimBot Result", "N/A");
//        }
        MainBot.shared.telemetry.addData("Goal Distance", distance);
        MainBot.shared.telemetry.addData("Goal Horizontal Distance", horizontal);
        MainBot.shared.telemetry.addData("Goal RAW Angle Error", rawAngleError);
        MainBot.shared.telemetry.addData("Goal Angle Error", angleError);
    }

    public class AimBotAction implements Action {

        private AimBot aimBot;
        private boolean firstLoop = true;
        private long lastNotAlignedTime;

        private AimBotAction(AimBot aimBot) {
            this.aimBot = aimBot;
            lastNotAlignedTime = System.currentTimeMillis();
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
            double turnPower = getTurnPower();

            MainBot.shared.moveBotMecanum(0, turnPower, 0, 1);

            // TRUE if aiming
            // FALSE if stopping
            boolean turnCondition = turnPower != 0;

            if (turnCondition) {
                // we're still aligning
                lastNotAlignedTime = System.currentTimeMillis();
            }

            // TRUE if it's been too soon since we were last misaligned
            // FALSE if it's been long enough since we were last misaligned
            boolean timeCondition = (System.currentTimeMillis() - lastNotAlignedTime) < AimBot.MIN_FOUND_TIME;

            telemetryPacket.addLine("Time Condition " + timeCondition);
            telemetryPacket.addLine("Turn Power Condition " + turnCondition);

            // TRUE if still aiming
            // FALSE if properly aimed
            return timeCondition || turnCondition;
        }
    }

    public AimBotAction getAction() {
        return new AimBotAction(this);
    }
}
