package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher extends VelMotor {
    public static double LAUNCH_SPEED = 6000; // rpm
    public static double MIN_LAUNCH_SPEED = 5600; // rpm

    public static double SERVO_DOWN_POS = 0.2;
    public static double SERVO_UP_POS = 0.05;
    public static int SERVO_WAIT = 1000; // ms

    private AimBot aimBot;

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", "kickServo", 27);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public Launcher(HardwareMap hardwareMap, AimBot aimBot) {
        super(hardwareMap, "launchMotor", "kickServo", 27);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.aimBot = aimBot;
    }

    public void setAimBot(AimBot aimBot) {
        this.aimBot = aimBot;
    }

    @Override
    public double getSpeed() {
        if (aimBot != null) {
            if (aimBot.foundGoal) {
                double launchPower = aimBot.getLaunchPower();
                if (launchPower > 0.0) {
                    return launchPower * 6000; // 0-1 power -> 0-6000rpm
                }
            }
        }
        return LAUNCH_SPEED;
    }

    @Override
    public double getMinSpeed() {
        return MIN_LAUNCH_SPEED;
    }

    public static double getServoUpPos() {
        return SERVO_UP_POS;
    }

    public static double getServoDownPos() {
        return SERVO_DOWN_POS;
    }
}
