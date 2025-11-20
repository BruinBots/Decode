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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;

import java.util.ArrayList;

@Config
@Autonomous(name="SixFarAuto")
public class SixFarAuto extends OpMode {
    private enum State {
        INIT,
        DRIVING_TO_LAUNCH,
        AIMING,
        LAUNCH0,
        LAUNCH1,
        LAUNCH2,
        PICKING,
        AIMING2,
        LAUNCH3,
        LAUNCH4,
        LAUNCH5,
        PARKING,
        END,
    }

    private State currentState = State.INIT;
    private MainBot bot;
    private AimBot aimBot;
    private MecanumDrive drive;
    private TrajectoryActionBuilder builder;
    private ArrayList<Action> actions = new ArrayList<>();
    private FtcDashboard dashboard;

    public static double GOAL_X = 54;
    public static double GOAL_Y = -18;
    public static double GOAL_HEADING = 205; // degrees

    public static double PICK_X = -36;
    public static double PICK_Y = -24;
    public static double PARK_X = -48;
    public static double PARK_Y = -18;

    public static double HYBRID_BRAKE_TIME = 2;
    public static double EVERY_X_BRAKE = 2;
    public static double REVERSE_BRAKE_POWER = 0;
    private Rotation2d lastLaunchAngle;

    private Pose2d getStartPose() {
        return new Pose2d(60, -12, Math.toRadians(180));
    }

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        aimBot = new AimBot();
        drive = bot.drive;
        drive.localizer.setPose(getStartPose());
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // State machine
        if (currentState == State.INIT) {
            // Drive to launcher
            actions.add(bot.drive.actionBuilder(getStartPose())
                .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(GOAL_HEADING))
                .afterTime(0.1, bot.launcher.getPowerAction(AimBot.FAR_POWER))
                .build());
            currentState = State.DRIVING_TO_LAUNCH;
        } else if (currentState == State.DRIVING_TO_LAUNCH) {
            // Aimbot
            if (actions.isEmpty()) {
                actions.add(aimBot.getAction());
                currentState = State.AIMING;
            }
        } else if (currentState == State.AIMING) {
            // Launch
            if (actions.isEmpty()) {
                lastLaunchAngle = bot.drive.localizer.getPose().heading;
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH0;
            }
        } else if (currentState == State.LAUNCH0) {
            // Launch
            if (actions.isEmpty()) {
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH1;
            }
        } else if (currentState == State.LAUNCH1) {
            // Launch
            if (actions.isEmpty()) {
                actions.add(new SequentialAction(
                        bot.singleLaunchAction(aimBot.getLaunchPower()),
                        bot.launcher.getPowerAction(0)
                ));
                currentState = State.LAUNCH2;
            }
        } else if (currentState == State.LAUNCH2) {
            if (actions.isEmpty()) {
                actions.add(drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(PICK_X, PICK_Y, Math.toRadians(270)), Math.toRadians(270))
                        .afterTime(0.1, new ParallelAction(
                                bot.intake.getPowerAction(Intake.INTAKE_POWER),
                                new Action() {
                                    private int iter = 0;
                                    private long startTime;
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        if (iter == 0) {
                                            startTime = System.currentTimeMillis();
                                        }
                                        if (iter % EVERY_X_BRAKE == 0) {
                                            bot.launcher.motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                            bot.launcher.spinUp(-REVERSE_BRAKE_POWER);
                                        } else {
                                            bot.launcher.spinUp(0);
                                        }
                                        bot.launcher.motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                                        iter ++;
                                        return System.currentTimeMillis() - startTime < HYBRID_BRAKE_TIME;
                                    }
                                }))
                        .lineToY(PICK_Y-IntakeAuto.DISTANCE, new TranslationalVelConstraint(IntakeAuto.VELOCITY))
                        .afterTime(0.1, new ParallelAction(
                                bot.intake.getPowerAction(0),
                                bot.launcher.getPowerAction(AimBot.FAR_POWER)
                        ))
//                        .lineToY(PICK_Y)
                        .strafeToLinearHeading(getStartPose().position, Math.toRadians(225))
                        .build()
                );
                currentState = State.PICKING;
            }
        } else if (currentState == State.PICKING) {
            if (actions.isEmpty()) {
                actions.add(aimBot.getAction());
                actions.add(drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(lastLaunchAngle) // fast adjust
//                        .turnTo(lastLaunchAngle, new TurnConstraints(Math.PI / 2, -Math.PI / 2, Math.PI / 2)) // slow fine tune
                        .build());
                currentState = State.AIMING2;
            }
        } else if (currentState == State.AIMING2) {
            if (actions.isEmpty()) {
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH3;
            }
        } else if (currentState == State.LAUNCH3) {
            if (actions.isEmpty()) {
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH4;
            }
        } else if (currentState == State.LAUNCH4) {
            if (actions.isEmpty()) {
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH5;
            }
        } else if (currentState == State.LAUNCH5) {
            if (actions.isEmpty()) {
                builder = drive.actionBuilder(drive.localizer.getPose())
                        .afterDisp(1, bot.launcher.getPowerAction(0))
                        .strafeToLinearHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(225));
                actions.add(builder.build());
                currentState = State.PARKING;
            }
        } else if (currentState == State.PARKING) {
            if (actions.isEmpty()) {
                currentState = State.END;
                requestOpModeStop();
                return;
            }
        }
        // Loop actions
        TelemetryPacket packet = new TelemetryPacket();
        ArrayList<Action> newActions = new ArrayList<>();
        for (Action action: actions) {
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        actions = newActions;
        bot.telemetry.addData("State", currentState);
        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
