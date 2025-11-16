package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.Autonomous.BaseAuto;
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
        LAUNCHING,
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

    private Pose2d getStartPose() {
        return new Pose2d(-60, -36, Math.toRadians(270));
    }

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        aimBot = new AimBot();
        drive = new MecanumDrive(hardwareMap, getStartPose());
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // State machine
        if (currentState == State.INIT) {
            builder = drive.actionBuilder(getStartPose())
                    .strafeToLinearHeading(new Vector2d(GOAL_X, GOAL_Y), Math.toRadians(225));
            actions.add(builder.build());
            currentState = State.DRIVING_TO_LAUNCH;
        } else if (currentState == State.DRIVING_TO_LAUNCH) {
            if (actions.isEmpty()) {
                actions.add(aimBot.getAction());
                currentState = State.AIMING;
            }
        } else if (currentState == State.AIMING) {
            if (actions.isEmpty()) {
                actions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
                currentState = State.LAUNCHING;
            }
        }
        // TODO: launching
        // TODO: parking
        // TODO: end
        // Loop actions
        TelemetryPacket packet = new TelemetryPacket();
        ArrayList<Action> newActions = new ArrayList<>();
        for (Action action: actions) {
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        actions = newActions;
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.addData("State", currentState);
        bot.telemetry.update();
    }
}
