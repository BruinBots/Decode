//package org.firstinspires.ftc.teamcode.Components;
//
//import android.sax.StartElementListener;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.sun.tools.javac.Main;
//
//import org.firstinspires.ftc.teamcode.MainBot;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import java.util.List;
//
//@Config
//public class QuickAimBot {
//    /*
//    Locate the april tag, then locate a point X_BEHIND inches behind it
//    for aiming with Roadrunner (faster than normal AimBot).
//     */
//    public static double CLOSE_POWER = 0.71;
//    public static double THRESHOLD_DISTANCE = 72.0;
//    public static double FAR_POWER = 0.9;
//    public static double X_BEHIND = 6.0;
//
//    public Vector2d behindPoint;
//    public double distance;
//
//    public QuickAimBot() {
//        behindPoint = new Vector2d(0, 0);
//        distance = 0;
//    }
//
//    public void readAprilTag() {
//        List<AprilTagDetection> currentDetections = MainBot.shared.aprilTags.readAprilTags();
//
//        MainBot.shared.drive.updatePoseEstimate();
//        Pose2d curPose = MainBot.shared.drive.localizer.getPose();
//        Vector2d curPos = curPose.position;
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
//                // Tag pos relative to robot (+x right, +y forward)
//                Vector2d rel = new Vector2d(detection.ftcPose.x, detection.ftcPose.y);
//                double h = curPose.heading.toDouble();
//                double cos = Math.cos(h);
//                double sin = Math.sin(h);
//
//                // Rotated rel by the robot's heading
//                Vector2d tagFieldOffset = new Vector2d(
//                        rel.x * cos - rel.y * sin,
//                        rel.x * sin + rel.y * cos
//                );
//
//                // Absolute tag pos on the field
//                Vector2d tagFieldPos = new Vector2d(
//                        curPose.position.x + tagFieldOffset.x,
//                        curPose.position.y + tagFieldOffset.y
//                );
//
//                // Determine normal vector directly behind the tag
//                // yaw is positive in the counterclockwise direction
//                // when viewed from above.
//                // Yaw is 0 when looking perpendicular to the tag
//                // (head on)
//                double normalRobotAngleBehind = detection.ftcPose.yaw + Math.PI/2;
//                Vector2d normalRobot = new Vector2d(
//                        Math.cos(normalRobotAngleBehind),
//                        Math.sin(normalRobotAngleBehind)
//                );
//                // Normal vector behind the tag in field coordinates
//                Vector2d normalField = new Vector2d(
//                        normalRobot.x * cos - normalRobot.y * sin,
//                        normalRobot.x * sin + normalRobot.y * cos
//                );
//
//                // Point behind the april tag in field coordinates (used for aiming)
//                behindPoint = new Vector2d(
//                        tagFieldPos.x + normalField.x * X_BEHIND,
//                        tagFieldPos.y + normalField.y * X_BEHIND
//                );
//
//                MainBot.shared.telemetry.addData("Relative Pos", rel.x+","+rel.y);
//                MainBot.shared.telemetry.addData("Heading (º)", Math.toDegrees(h));
//                MainBot.shared.telemetry.addData("Cos(heading)", cos);
//                MainBot.shared.telemetry.addData("Sin(heading)", sin);
//                MainBot.shared.telemetry.addData("Tag Field Offset", tagFieldOffset.x+","+tagFieldOffset.y);
//                MainBot.shared.telemetry.addData("Tag Field Pose", tagFieldPos.x+","+tagFieldPos.y);
//                MainBot.shared.telemetry.addData("Normal Angle Behind", normalRobotAngleBehind);
//                MainBot.shared.telemetry.addData("Normal Robot", normalField.x+","+normalField.y);
//                MainBot.shared.telemetry.addData("Normal Field", normalField.x+","+normalField.y);
//            }
//        }
//
//        MainBot.shared.telemetry.addData("Behind Point", behindPoint.x+","+behindPoint.y);
//        // find distance to goal
//        distance = behindPoint.minus(curPos).norm();
//    }
//
//    public boolean didFindGoal() {
//        return distance > 0 && behindPoint.x != 0 && behindPoint.y != 0;
//    }
//
//    public double getAngleToGoal() {
//        MainBot.shared.drive.updatePoseEstimate();
//        Vector2d curPose = MainBot.shared.drive.localizer.getPose().position;
//        Vector2d delta = behindPoint.minus(curPose);
//        return delta.angleCast().toDouble() - (Math.PI / 2);
//    }
//
//    public double getLaunchPower() {
//        if (distance < THRESHOLD_DISTANCE) {
//            return CLOSE_POWER;
//        } else if (distance > THRESHOLD_DISTANCE) {
//            return FAR_POWER;
//        } else {
//            return CLOSE_POWER;
//        }
//    }
//
//    public Action aimAction() {
//        // Returns a RR action to turn to the goal
//        readAprilTag();
//        if (didFindGoal()) {
//            return MainBot.shared.drive.actionBuilder(MainBot.shared.drive.localizer.getPose())
//                .turnTo(getAngleToGoal())
//                .build();
//        }
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                telemetryPacket.addLine("Not performing QuickAimBot");
//                return false;
//            }
//        };
//    }
//}
