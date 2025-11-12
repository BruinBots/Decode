package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="IntakeAuto", group="Testing")
public class IntakeAuto extends OpMode {
    private MainBot bot;
    private MecanumDrive drive;
    private Action action;

    public static double DISTANCE = 24;
    public static double VELOCITY = 10;

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        action = new SequentialAction(
                bot.intake.getPowerAction(Intake.INTAKE_POWER),
                drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(DISTANCE, new TranslationalVelConstraint(VELOCITY))
                    .build(),
                bot.intake.getPowerAction(0)
        );
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        if (!action.run(packet)) {
            stop();
        }
        bot.dashboard.sendTelemetryPacket(packet);
        bot.telemetry.update();
    }
}
