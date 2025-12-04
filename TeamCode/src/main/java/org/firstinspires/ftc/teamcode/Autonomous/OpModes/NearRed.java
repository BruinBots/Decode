package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.AllLaunchAction;

@Autonomous(name="NearRed")
@Config
public class NearRed extends OpMode {
    private MainBot bot;
    private MecanumDrive drive;
    private FtcDashboard dashboard;

    public static double GOAL_X = NearBlue.GOAL_X;
    public static double GOAL_Y = -NearBlue.GOAL_Y;

    public static double PICK_X = NearBlue.PICK_X;
    public static double PICK_Y = -NearBlue.PICK_Y;
    public static double PARK_X = NearBlue.PARK_X;
    public static double PARK_Y = -NearBlue.PARK_Y;

    public static double GOAL_ANGLE = 360-NearBlue.GOAL_ANGLE;

    public static double WAIT_SECONDS = NearBlue.WAIT_SECONDS;

    public Action action;

    public Pose2d getStartPose() {
        return new Pose2d(-60, 36, Math.toRadians(90));
    }
    public PoseMap getPoseMap() {
        return new PoseMap() {
            @NonNull
            @Override
            public Pose2dDual<Arclength> map(@NonNull Pose2dDual<Arclength> pose2dDual) {
                return pose2dDual;
            }
        };
    }

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        drive = bot.drive;
        drive.localizer.setPose(getStartPose());
        dashboard = FtcDashboard.getInstance();

        TrajectoryActionBuilder driveToLaunch1 = drive.actionBuilder(getStartPose(), getPoseMap())
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE));

        TrajectoryActionBuilder pick = driveToLaunch1.endTrajectory().fresh()
                .waitSeconds(WAIT_SECONDS)
                .setTangent(180)
                .afterTime(0.1, new ParallelAction(
                        bot.intake.getPowerAction(Intake.INTAKE_POWER),
                        bot.launcher.getPowerAction(-Launcher.REVERSE_POWER)
                ))
                .splineToSplineHeading(new Pose2d(PICK_X, PICK_Y, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(PICK_Y+IntakeAuto.DISTANCE, new TranslationalVelConstraint(IntakeAuto.VELOCITY))
                .lineToY(PICK_Y);

        TrajectoryActionBuilder driveToLaunch2 = pick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE));

        TrajectoryActionBuilder park = driveToLaunch2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(PARK_X, PARK_Y));

        action = new SequentialAction(
                telemetryPacket -> {
                    bot.launcher.motor.setTargetVelocity(AimBot.CLOSE_VELOCITY);
                    return false;
                },
                driveToLaunch1.build(),
                new AllLaunchAction(AimBot.CLOSE_VELOCITY),
                pick.build(),
                telemetryPacket -> {
                    bot.launcher.motor.setTargetVelocity(AimBot.CLOSE_VELOCITY);
                    return false;
                },
//                bot.launcher.getPowerAction(AimBot.CLOSE_POWER),
                driveToLaunch2.build(),
                bot.launcher.getPowerAction(0),
                park.build()
        );
    }

    @Override
    public void loop() {
        // Loop action
        TelemetryPacket packet = new TelemetryPacket();
        if (!action.run(packet)) {
            requestOpModeStop();
        }
        bot.launcher.doTelemetry();
        bot.launcher.motor.updateVelocityPID();
        bot.intake.doTelemetry();
        bot.voltageCompensator.doTelemetry();
        bot.drive.updatePoseEstimate();
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
