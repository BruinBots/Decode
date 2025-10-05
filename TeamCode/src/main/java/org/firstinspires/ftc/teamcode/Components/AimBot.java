package org.firstinspires.ftc.teamcode.Components;

import static android.os.Build.VERSION_CODES.R;
import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AimBot implements SBA {
    private static AprilTagProcessor aprilTag;
    private static VisionPortal visionPortal;

    // April tag status variables
    public static boolean foundGoal = false;
    public static double distance = 0.0;
    public static  double angleError = 180.0;

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

    public static void initVisionPortal(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }

    public static void stop() {
        // Close vision on a separate thread to avoid blocking the OpMode stop sequence
//        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
        visionPortal.close();
    }

    public static double getLaunchPower() {
        if (distance < MIN_DISTANCE || distance > FAR_DISTANCE) {
            return 0.0;
        } else if (distance < CLOSE_DISTANCE) {
            return CLOSE_POWER;
        } else {
            double slope = (FAR_POWER-CLOSE_POWER)/(FAR_DISTANCE-CLOSE_DISTANCE);
            return CLOSE_POWER + slope*(distance-CLOSE_DISTANCE);
        }
    }

    public static double getTurnPower() {
        if (Math.abs(angleError) > 30.0) { return 0.0; }
        if (Math.abs(angleError) < 5.0) { return 0.0; }
        double power = -TURN_kP * angleError;
        if (Math.abs(power) < TURN_MIN_POWER) {
            return Math.copySign(TURN_MIN_POWER, power);
        } else if (Math.abs(power) > TURN_MAX_POWER) {
            return Math.copySign(TURN_MAX_POWER, power);
        } else {
            return power;
        }
    }

    public void preInit() {
//        initAprilTag();
        visionPortal.resumeStreaming();
    }

    public void init() { }

    public boolean sanity() {
        readAprilTag();
        return foundGoal && distance >= 10.0 && distance <= 100.0 && Math.abs(angleError) <= 30.0;
    }

    public void loop() {
        readAprilTag();
        double launchPower = getLaunchPower();
        double turnPower = getTurnPower();
        if (foundGoal) {
            MainBot.shared.telemetry.addData("Goal Distance", distance);
            MainBot.shared.telemetry.addData("Goal Angle Error", angleError);
        } else {
            MainBot.shared.telemetry.addData("Goal Distance", "N/A");
            MainBot.shared.telemetry.addData("Goal Angle Error", "N/A");
        }
        MainBot.shared.telemetry.addData("Launch Power", launchPower);
        MainBot.shared.telemetry.addData("Turn Power", turnPower);
        MainBot.shared.telemetry.update();
    }

    public boolean isBusy() {
         boolean isBusy = !foundGoal || Math.abs(angleError) >= 2.0;
         if (!isBusy) {
//             visionPortal.stopStreaming();
//             visionPortal.close();
         }
         return isBusy;
    }

    private static void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }
    private void readAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        foundGoal = false;
        distance = 0.0;
        angleError = 0.0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                distance = detection.ftcPose.y;
                angleError = detection.ftcPose.yaw;
                foundGoal = true;
            }
        }
    }
}
