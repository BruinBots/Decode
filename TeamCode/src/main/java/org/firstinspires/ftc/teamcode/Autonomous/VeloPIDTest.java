package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.MainBot;

@TeleOp(name="VeloPIDTest")
@Config
public class VeloPIDTest extends OpMode {
    public static double TARGET_SPEED = 0;

//    public static double kP = 0;
//    public static double kI = 0;
//    public static double kD = 0;

    /*
    rpm adjustment
    input, output
    4300, 3700
    5300, 4550
     */

    public static boolean kickUp = false;

    private MainBot bot;

    @Override
    public void init() {
        bot = MainBot.shared = new MainBot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
//        bot.launcher.motor.setVelPID(kP, kI, kD);
        bot.launcher.motor.setTargetVelocity(TARGET_SPEED);
        if (TARGET_SPEED == 0) {
            bot.launcher.spinUp(0);
        }
        bot.launcher.motor.updateVelocityPID();
        bot.launcher.doTelemetry();
        if (kickUp) {
            bot.launcher.setServo(Launcher.SERVO_UP_POS);
        } else {
            bot.launcher.setServo(Launcher.SERVO_DOWN_POS);
        }
        MainBot.shared.telemetry.update();
    }
}
