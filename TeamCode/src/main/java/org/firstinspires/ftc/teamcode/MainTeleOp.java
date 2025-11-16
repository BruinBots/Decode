package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
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
    public ArrayList<Action> launchActions;
    public ArrayList<Action> driveActions;

    public static double DRIVE_FACTOR = 0.6;

    public AimBot aimBot;
    public ObeliskReader obeliskReader;

    public FtcDashboard dash;

    private boolean didAddAimBotAction = false;
    private boolean didAddSingleLaunchAction = false;
    private boolean didAddIntakeDriveAction = false;
    private boolean isLaunching = false;
//    private boolean isTestingLaunchPID = false;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        bot = MainBot.shared;

        aimBot = new AimBot();
        obeliskReader = new ObeliskReader();

        dash = FtcDashboard.getInstance();

        actions = new ArrayList<>();
        launchActions = new ArrayList<>();
        driveActions = new ArrayList<>();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            launchActions.clear();
            bot.launcher.setServo(Launcher.SERVO_UP_POS);
        }
        else if (launchActions.isEmpty()) {
            bot.launcher.setServo(Launcher.SERVO_DOWN_POS);
        }

//        if (gamepad1.dpad_up) {
//            bot.intake.kickUp();
//        } else if (gamepad1.dpad_down) {
//            bot.intake.kickDown();
//        }

        if (gamepad1.a) {
            launchActions.clear();
            bot.intake.setPower(Intake.INTAKE_POWER);
            if (!bot.launcher.isActive()) {
                bot.launcher.spinUp(-Launcher.REVERSE_POWER);
            }
        } else if (gamepad1.b) {
            launchActions.clear();
            bot.intake.setPower(-Intake.REVERSE_POWER);
            if (!bot.launcher.isActive()) {
                bot.launcher.spinUp(Launcher.REVERSE_POWER);
            }
        } else {
            if (launchActions.isEmpty() && driveActions.isEmpty()) {
                bot.intake.doStop();
                if (!isLaunching) {
                    bot.launcher.doStop();
                }
             }
        }

        if (gamepad1.x) {
            launchActions.clear();
            double power = Launcher.LAUNCH_POWER;
            if (aimBot.foundGoal) {
                power = aimBot.getLaunchPower();
            }
            bot.launcher.spinUp(power);
            isLaunching = true;
        } else if (gamepad1.y) {
            launchActions.clear();
            bot.launcher.doStop();
            isLaunching = false;
        }

        if (gamepad1.right_bumper && gamepad1.dpad_right && !didAddAimBotAction) {
            driveActions.add(aimBot.getAction());
            didAddAimBotAction = true;
        } else {
            didAddAimBotAction = false;
        }

        if (gamepad1.right_trigger > 0.8 && !didAddSingleLaunchAction) {
            launchActions.add(bot.singleLaunchAction(aimBot.getLaunchPower()));
            didAddSingleLaunchAction = true;
        } else {
            didAddSingleLaunchAction = false;
        }

        if (gamepad1.dpad_left && !didAddIntakeDriveAction) {
            driveActions.add(bot.intakeDriveAction());
            didAddIntakeDriveAction = true;
        } else {
            didAddIntakeDriveAction = false;
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

        bot.launcher.cookedMotor.loop(gamepad1);
        bot.intake.cookedMotor.loop(gamepad1);

        aimBot.readAprilTag();
        aimBot.doTelemetry();

        bot.telemetry.addData("Obelisk", obeliskReader.read().toString());

        actions = actionLoop(actions);
        launchActions = actionLoop(launchActions);
        driveActions = actionLoop(driveActions);

//        TelemetryPacket packet = new TelemetryPacket();
//        ArrayList<Action> newActions = new ArrayList<>();
//        for (Action action : actions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        actions = newActions;
//
//        ArrayList<Action> newDriveActions = new ArrayList<>();
//        for (Action action : driveActions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newDriveActions.add(action);
//            }
//        }
//        driveActions = newDriveActions;

        telemetry.update();
        bot.telemetry.update();

        bot.drive.updatePoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();
        bot.drive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, bot.drive.localizer.getPose());
        bot.dashboard.sendTelemetryPacket(packet);

        // NOTE: Left stick y is negative when pushed forward (up) on PS4 controller
        double drive = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        if (drive != 0 || strafe != 0 || rotate != 0) {
            driveActions.clear();
        }
        if (driveActions.isEmpty()) {
            bot.moveBotMecanum(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, DRIVE_FACTOR);
        }
    }

    public ArrayList<Action> actionLoop(ArrayList<Action> actionList) {
        ArrayList<Action> newActions = new ArrayList<>();
        TelemetryPacket packet = new TelemetryPacket();
        for (Action action : actionList) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        bot.dashboard.sendTelemetryPacket(packet);
        return newActions;
    }
}
