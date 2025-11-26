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

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.AllLaunchAction;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;

import java.util.ArrayList;
import java.util.Vector;

@Config
@Autonomous(name="SixFarBlue")
public class SixFarAuto extends OpMode {
    private MainBot bot;
    private MecanumDrive drive;
    private FtcDashboard dashboard;

    public static double GOAL_X = 54;
    public static double GOAL_Y = -18;
    public static double GOAL_HEADING = 195; // degrees

    public static double PICK_X = 42;
    public static double PICK_Y = -18;
    public static double PARK_X = 48;
    public static double PARK_Y = -18;

    public static double HYBRID_BRAKE_TIME = 2;
    public static double EVERY_X_BRAKE = 2;
    public static double REVERSE_BRAKE_POWER = 0;

    private Action action;
    private Pose2d getStartPose() {
        return new Pose2d(60, -12, Math.toRadians(180));
    }

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        drive = bot.drive;
        drive.localizer.setPose(getStartPose());
        dashboard = FtcDashboard.getInstance();

        TrajectoryActionBuilder driveToLaunch1 = drive.actionBuilder(getStartPose())
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_HEADING));

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
                .afterTime(0.1, new ParallelAction(
                        bot.intake.getPowerAction(0),
                        bot.launcher.getPowerAction(AimBot.FAR_POWER),
                        telemetryPacket -> {
                            bot.launcher.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            return false;
                        }
                ))
//                        .lineToY(PICK_Y)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(GOAL_X, GOAL_Y, Math.toRadians(GOAL_HEADING)), Math.toRadians(2));

        TrajectoryActionBuilder park = pick.endTrajectory().fresh()
                .afterDisp(1, bot.launcher.getPowerAction(0))
                .strafeToLinearHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(225));

        action = new SequentialAction(
                bot.launcher.getPowerAction(AimBot.FAR_POWER),
                driveToLaunch1.build(),
                new AllLaunchAction(AimBot.FAR_POWER),
                pick.build(),
                pick.endTrajectory().fresh().turnTo(Math.toRadians(GOAL_HEADING), new TurnConstraints(Math.PI/2, -Math.PI/2, Math.PI/2)).build(),
                new AllLaunchAction(AimBot.FAR_POWER),
                park.build()
        );

    }

    @Override
    public void loop() {
        // Loop actions
        TelemetryPacket packet = new TelemetryPacket();
        if (!action.run(packet)) {
            requestOpModeStop();
        }
        drive.updatePoseEstimate();
        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
