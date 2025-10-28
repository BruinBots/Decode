package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.Vector;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 600, Math.toRadians(360), Math.toRadians(3600), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -12, 0))
                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
                        // Launch
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(-10)))
//                        // Read obelisk, spin up intake (max 3s), also try panning to find the obelisk
                        .setReversed(false)
//                        // PGP
//                        .splineTo(new Vector2d(-12, -36), Math.toRadians(270))
//                        .forward(12)
//                        .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(-42, -36, Math.toRadians(225)), Math.toRadians(225))
//                        // PGP
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(12, -36), Math.toRadians(270))
//                        .forward(12)
//                        .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(0, -18, Math.toRadians(270)), Math.toRadians(180))
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
//                        // PPG
//                        .turn(Math.toRadians(180))
                        .splineTo(new Vector2d(36, -36), Math.toRadians(270))
                        .forward(12)
                        .setReversed(true)
//                        .back(12)
                        .splineToLinearHeading(new Pose2d(18, -18, Math.toRadians(180)), Math.toRadians(180))
                        .setReversed(false)
                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
//                        // Score again
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
                        .build());


//        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
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