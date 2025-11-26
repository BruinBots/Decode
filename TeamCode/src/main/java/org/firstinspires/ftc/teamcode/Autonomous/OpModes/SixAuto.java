package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.AllLaunchAction;

import java.util.ArrayList;

@Config
@Autonomous(name="SixBlue")
public class SixAuto extends OpMode {
    private MainBot bot;
    private MecanumDrive drive;
    private FtcDashboard dashboard;

    public static double GOAL_X = -18;
    public static double GOAL_Y = -12;

    public static double PICK_X = -12;
    public static double PICK_Y = -24;
    public static double PARK_X = -38;
    public static double PARK_Y = -24;

    public static double GOAL_ANGLE = 231;

    public static double HYBRID_BRAKE_TIME = 2;
    public static double BRAKE_PERCENT = 0.85;
    public static double REVERSE_BRAKE_POWER = 0;

    public Action action;

    private Pose2d getStartPose() {
        return new Pose2d(-60, -36, Math.toRadians(270));
    }

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        drive = bot.drive;
        drive.localizer.setPose(getStartPose());
        dashboard = FtcDashboard.getInstance();

        TrajectoryActionBuilder driveToLaunch1 = drive.actionBuilder(getStartPose())
                .afterDisp(1, bot.launcher.getPowerAction(AimBot.CLOSE_POWER))
                .afterDisp(1, bot.launcher.getPowerAction(AimBot.CLOSE_POWER))
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE));

        TrajectoryActionBuilder pick = driveToLaunch1.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(PICK_X, PICK_Y, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0.1, new ParallelAction(
                        bot.intake.getPowerAction(Intake.INTAKE_POWER),
                        telemetryPacket -> {
                            bot.launcher.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            bot.launcher.spinUp(0);
                            return false;
                        }))
                .lineToY(PICK_Y-IntakeAuto.DISTANCE, new TranslationalVelConstraint(IntakeAuto.VELOCITY))
                .afterTime(0.1, bot.launcher.getPowerAction(AimBot.CLOSE_POWER))
                .afterTime(1.5, bot.intake.getPowerAction(0))
//                        .lineToY(PICK_Y)
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE));

        TrajectoryActionBuilder park = pick.endTrajectory().fresh()
                .afterDisp(1, bot.launcher.getPowerAction(0))
                .strafeToLinearHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(225));

        action = new SequentialAction(
                driveToLaunch1.build(),
                new AllLaunchAction(AimBot.CLOSE_POWER),
                pick.build(),
                telemetryPacket -> {
                    bot.launcher.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    return false;
                },
                new AllLaunchAction(AimBot.CLOSE_POWER),
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
        bot.intake.doTelemetry();
        bot.voltageCompensator.doTelemetry();
        bot.drive.updatePoseEstimate();
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
