package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.ObeliskReader;
import org.firstinspires.ftc.teamcode.Components.WaitAction;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class BaseAuto {
    private MecanumDrive drive;

    private MainBot bot;
    private AimBot aimBot;
    private ObeliskReader obeliskReader;

    public BaseAuto() {
        drive = new MecanumDrive(MainBot.shared.hardwareMap, getStartPose());
        bot = MainBot.shared;
        aimBot = new AimBot();
        obeliskReader = new ObeliskReader();
    }

    public Pose2d getStartPose() {
        return new Pose2d(-60, 12, 0);
    }

    public class TrajectoryEnd {
        public Action action;
        public TrajectoryActionBuilder builder;

        public TrajectoryEnd(Action action, TrajectoryActionBuilder builder) {
            this.action = action;
            this.builder = builder;
        }
    }

    public TrajectoryEnd launch(Pose2d startPose) {
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
                .afterTime(0, bot.launcher.getSpinUpAction())
                .splineTo(new Vector2d(-42, -36), Math.toRadians(225));
        return new TrajectoryEnd(
                new SequentialAction(
                        builder.build(),
                        new ParallelAction(
                                aimBot.getAction(),
                                bot.launcher.getSpinUpAction()
                        ),
                        bot.launcher.servoUpAction(),
                        new WaitAction(1000),
                        bot.launcher.servoDownAction()
                ),
                builder.endTrajectory()
        );
    }

    public TrajectoryEnd obeliskRead(Pose2d startPose) {
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-12, -18), Math.toRadians(-20))
                .setReversed(false);
        return new TrajectoryEnd(
                new SequentialAction(
                        builder.build(),
                        obeliskReader.getReadAction()
                ),
                builder.endTrajectory()
        );
    }

    // Intake paths
    // TODO: Replace Action with TrajectoryEnd
    public Action GPPIntake(Pose2d startPose) {
        return drive.actionBuilder(startPose)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(-12, -36), Math.toRadians(270))
                .lineToY(-48)
                .setReversed(true)
                .lineToY(-36)
                .splineTo(new Vector2d(0, -18), Math.toRadians(0))
                .setReversed(false)
                .build();
    }

    public Action PGPIntake(Pose2d startPose) {
        return drive.actionBuilder(startPose)
                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(12, -36), Math.toRadians(270))
                .lineToY(-48)
                .setReversed(true)
                .lineToY(-36)
                .splineTo(new Vector2d(24, -18), Math.toRadians(0))
                .setReversed(false)
                .build();
    }

    public Action PPGIntake(Pose2d startPose) {
        return drive.actionBuilder(startPose)
                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(36, -36), Math.toRadians(270))
                .lineToY(-48)
                .setReversed(true)
                .lineToY(-36)
                .splineTo(new Vector2d(48, -18), Math.toRadians(0))
                .setReversed(false)
                .build();
    }
}
