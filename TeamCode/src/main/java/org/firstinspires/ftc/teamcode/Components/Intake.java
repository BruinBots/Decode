package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Intake extends VelMotor {

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_SPEED = 100; // rpm
    public static double MIN_INTAKE_SPEED = 80; // rpm

    public static double SERVO_DOWN_POS = 0.15;
    public static double SERVO_UP_POS = 0.5;

    public static double REVERSE_POWER = 0.3;

    private Servo servo;

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeMotor", "intakeServo", -175);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
