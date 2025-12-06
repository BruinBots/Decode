package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
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

@Autonomous(name="FarBlue")
@Config
public class FarBlue extends OpMode {
    private MainBot bot;
    private MecanumDrive drive;
    private FtcDashboard dashboard;

    public static double GOAL_X = 54;
    public static double GOAL_Y = -18;

    public static double PICK_X = 40;
    public static double PICK_Y = -16;

    public static double PARK_X = 42;
    public static double PARK_Y = -12;

    public static double INTAKE_DISTANCE = 34;

    public static double GOAL_ANGLE = 199;

    public static double WAIT_SECONDS = 2;

    public Action action;

    public Pose2d getStartPose() {
        return new Pose2d(60, -12, Math.toRadians(180));
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
//                .setTangent(0)
                .afterTime(0.1, new ParallelAction(
                        bot.intake.getPowerAction(Intake.INTAKE_POWER),
                        bot.launcher.getPowerAction(-Launcher.REVERSE_POWER)
                ))
                .strafeToLinearHeading(new Vector2d(PICK_X, PICK_Y), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(PICK_X, PICK_Y-INTAKE_DISTANCE), new TranslationalVelConstraint(IntakeAuto.VELOCITY))
                .strafeToConstantHeading(new Vector2d(PICK_X, PICK_Y));

        TrajectoryActionBuilder driveToLaunch2 = pick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE), new AngularVelConstraint(Math.PI / 4.0));

        TrajectoryActionBuilder park = driveToLaunch2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(PARK_X, PARK_Y));

        action = new SequentialAction(
                telemetryPacket -> {
                    bot.launcher.motor.setTargetVelocity(AimBot.FAR_VELOCITY);
                    return false;
                },
                driveToLaunch1.build(),
                new AllLaunchAction(AimBot.FAR_VELOCITY),
                pick.build(),
                telemetryPacket -> {
                    bot.launcher.motor.setTargetVelocity(AimBot.FAR_VELOCITY);
                    return false;
                },
//                bot.launcher.getPowerAction(AimBot.CLOSE_POWER),
                driveToLaunch2.build(),
                new Action() { // correct last move
                    private Action action;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (action == null) {
                            action = drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_ANGLE), new AngularVelConstraint(Math.PI / 4.0))
                                    .build();
                        }
                        return action.run(telemetryPacket);
                    }
                },
                new AllLaunchAction(AimBot.FAR_VELOCITY, 2),
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
