package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher extends VelMotor {

    public static double LAUNCH_POWER = 0.8;
    public static double LAUNCH_SPEED = 6000; // rpm
    public static double MIN_LAUNCH_SPEED = 5600; // rpm

    public static double SERVO_DOWN_POS = 0.2;
    public static double SERVO_UP_POS = 0.05;

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", "kickServo", 27);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
