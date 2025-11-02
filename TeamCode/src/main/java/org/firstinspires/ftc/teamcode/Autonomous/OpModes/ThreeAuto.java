package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
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

    private Pose2d getStartPose() {
        return new Pose2d(-60, 12, 0);
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
                    .splineTo(new Vector2d(-42, -36), Math.toRadians(225));
            actions.add(builder.build());
            currentState = State.DRIVING_TO_LAUNCH;
        } else if (currentState == State.DRIVING_TO_LAUNCH) {
            if (actions.size() == 0) {
                actions.add(aimBot.getAction());
                currentState = State.AIMING;
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
    }
}
