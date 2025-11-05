package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Launcher extends VelMotor {

    public static double LAUNCH_POWER = 0.8;
    public static double LAUNCH_SPEED = 5600; // rpm

    public static double REVERSE_POWER = 0.05;

    public static double SERVO_DOWN_POS = 0.2;
    public static double SERVO_UP_POS = 0.38;

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", "kickServo", 28);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
