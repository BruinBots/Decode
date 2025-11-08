package org.firstinspires.ftc.teamcode.Components;

import android.app.Notification;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Launcher extends VelMotor {

    public static double LAUNCH_POWER = 0.7;
    public static double LAUNCH_SPEED = 3800; // rpm

    public static double REVERSE_POWER = 0.075;

    public static double SERVO_DOWN_POS = 0.2;
    public static double SERVO_UP_POS = 0.38;

    public static int SERVO_WAIT_MS = 1250;
    public static int POST_LAUNCH_WAIT_MS = 500;

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", "kickServo", 28);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public Action kickAction() {
        return new SequentialAction(
            getServoAction(SERVO_UP_POS),
            new WaitAction(SERVO_WAIT_MS),
            getServoAction(SERVO_DOWN_POS)
        );
    }
}
