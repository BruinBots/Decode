package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SBAs.SBAAction;

public class BaseAuto {
    private MecanumDrive drive;

    private MainBot bot;
    private AimBot aimBot;

    public BaseAuto() {
        drive = new MecanumDrive(MainBot.shared.hardwareMap, getStartPose());
        bot = MainBot.shared;
        aimBot = new AimBot();
    }

    public Pose2d getStartPose() {
        return new Pose2d(-60, 12, 0);
    }

    public Action firstLaunch(Pose2d startPose) {
        Action traj = drive.actionBuilder(startPose)
                .afterTime(0, bot.launcher.getSpinUpSBA().action())
                .splineTo(new Vector2d(-42, -36), Math.toRadians(225))
                .build();
        return new SequentialAction(
                traj,
                aimBot.getAimSBA().action(),
                new SBAAction(bot.launcher.kick())
        );
    }
}
