package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;

@Config
@Autonomous(name="ThreeAuto")
public class ThreeAuto extends OpMode {
    private enum State {
        INIT,
        DRIVING_TO_LAUNCH,
        AIMING,
        LAUNCH0,
        LAUNCH1,
        LAUNCH2,
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

    public static double GOAL_X = -26;
    public static double GOAL_Y = -24;
    public static double PARK_X = -38;
    public static double PARK_Y = -24;

    private Pose2d getStartPose() {
        return new Pose2d(-60, -36, Math.toRadians(270));
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
            builder = drive.actionBuilder(getStartPose())
                    .afterDisp(1, bot.launcher.getPowerAction(AimBot.CLOSE_POWER))
                    .afterDisp(1, bot.launcher.getPowerAction(AimBot.CLOSE_POWER))
                    .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(225));
            actions.add(builder.build());
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
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCH2;
            }
        } else if (currentState == State.LAUNCH2) {
            if (actions.isEmpty()) {
                builder = drive.actionBuilder(getStartPose())
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
