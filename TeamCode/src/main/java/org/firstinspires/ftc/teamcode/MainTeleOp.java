package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.ObeliskReader;

import java.util.ArrayList;

@Config
@TeleOp
public class MainTeleOp extends OpMode {
    public MainBot bot;
    public ArrayList<Action> actions;

    public static double DRIVE_FACTOR = 0.6;

    public AimBot aimBot;
    public ObeliskReader obeliskReader;

    public FtcDashboard dash;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;

        aimBot = new AimBot();
        obeliskReader = new ObeliskReader();

        dash = FtcDashboard.getInstance();

        actions = new ArrayList<>();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            bot.launcher.setServo(Launcher.SERVO_UP_POS);
        }
        else {
            bot.launcher.setServo(Launcher.SERVO_DOWN_POS);
        }

        if (gamepad1.dpad_up) {
            bot.intake.kickUp();
        } else if (gamepad1.dpad_down) {
            bot.intake.kickDown();
        }

        if (gamepad1.a) {
            bot.intake.spinUp();
        } else if (gamepad1.b) {
            bot.intake.reverse();
        } else {
            bot.intake.stop();
        }

        if (gamepad1.x) {
            bot.launcher.spinUp(Launcher.LAUNCH_POWER);
        } else if (gamepad1.y) {
            bot.launcher.doStop();
        }

        if (gamepad1.right_bumper && gamepad1.dpad_right) {
            actions.add(aimBot.getAction());
        }

//        if (gamepad1.left_bumper) {
//            bot.intake.spinUp(-Intake.REVERSE_POWER);
//        } else if (gamepad1.right_bumper) {
//            bot.intake.spinUp(Intake.INTAKE_POWER);
//        } else {
//            bot.intake.doStop();
//        }
//
//        if (gamepad1.y) {
//            // launch one
//        }
//
//        if (gamepad1.dpad_up) {
//            // henryjack up
//        } else if (gamepad1.dpad_down) {
//            // henryjack down
//        }
//
//        if (gamepad1.a) {
//            // clear current ball from launcher
//        }
//
//        if (gamepad1.x) {
//            // aim with apriltags
//        }
//
//        if (gamepad1.b) {
//            // launch all
//        }

        bot.launcher.doTelemetry();
        bot.intake.doTelemetry();

        bot.launcher.cookedMotor.loop();

        aimBot.readAprilTag();
        aimBot.doTelemetry();

        telemetry.addData("Obelisk", obeliskReader.read().toString());

        TelemetryPacket packet = new TelemetryPacket();
        ArrayList<Action> newActions = new ArrayList<>();
        for (Action action : actions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        actions = newActions;

        telemetry.update();

        bot.moveBotMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, DRIVE_FACTOR);
    }
}
