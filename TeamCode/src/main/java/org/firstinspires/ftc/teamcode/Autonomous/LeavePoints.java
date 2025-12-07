package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainBot;

@Config
@Autonomous(name="Leave")
public class LeavePoints extends OpMode {
    private Action action;
    private MainBot bot;
    private FtcDashboard dashboard;

    private static double X = 18;
    public static double Y = 18;

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot.drive.localizer.setPose(new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();

        action = bot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToConstantHeading(new Vector2d(X, Y))
                .build();
    }

    @Override
    public void loop() {
        // Loop action
        TelemetryPacket packet = new TelemetryPacket();
        if (!action.run(packet)) {
            requestOpModeStop();
        }
        bot.voltageCompensator.doTelemetry();
        bot.drive.updatePoseEstimate();
        dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
