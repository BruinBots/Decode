//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//
//import org.rowlandhall.meepmeep.MeepMeep;
//import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
//import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//import java.awt.Image;
//import java.io.File;
//import java.io.IOException;
//import java.io.InputStream;
//import java.util.Vector;
//
//import javax.imageio.ImageIO;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(600);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 600, Math.toRadians(360), Math.toRadians(3600), 15)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -12, 0))
//                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
//
//                        .build());
//
//
////        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
//        Image img = null;
//        ClassLoader classloader = Thread.currentThread().getContextClassLoader();
//        try {
//            InputStream is = classloader.getResourceAsStream("background/decode.png");
//            img = ImageIO.read(is);
//        }
//        catch(IOException e) {
//            System.out.println("Error: " + e);
//        }
//
//        meepMeep.setBackground(img)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//}

package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Image;
import java.io.IOException;
import java.io.InputStream;
import java.util.Vector;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -36, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-42, -36), Math.toRadians(225))
//                .lineToX(24, new TranslationalVelConstraint(10))
//                .strafeToLinearHeading(new Vector2d(-42, -36), Math.toRadians(225))
////                .stopAndAdd(new SequentialAction(
////                        aimBot.getAimAction(),
////                        bot.getSingleLaunchAction(),
////                        bot.getSingleLaunchAction(),
////                        bot.getSingleLaunchAction()
////                ))
//                .waitSeconds(5)
//                .strafeToLinearHeading(new Vector2d(-36, -16), Math.toRadians(175))
////                .stopAndAdd(obeliskReader.readObeliskAction())
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(270)), Math.toRadians(270))
////                .afterDisp(12, bot.intake.getServoAction(Intake.INTAKE_IN_POS))
//                .lineToY(-48)
//                .lineToY(-36)
////                .afterDisp(2, bot.intake.getServoAction(Intake.INTAKE_STOP_POS))
//                .splineToSplineHeading(new Pose2d(-42, -36, Math.toRadians(225)), Math.toRadians(225))
////                .stopAndAdd(new SequentialAction(
////                        aimBot.getAimAction(),
////                        bot.getSingleLaunchAction(),
////                        bot.getSingleLaunchAction(),
////                        bot.getSingleLaunchAction()
////                ))
//                .waitSeconds(5)
                .build());

//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();

        Image img = null;
        ClassLoader classloader = Thread.currentThread().getContextClassLoader();
        try {
            InputStream is = classloader.getResourceAsStream("background/decode.png");
            img = ImageIO.read(is);
        }
        catch(IOException e) {
            System.out.println("Error: " + e);
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}