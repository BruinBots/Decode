package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.teamcode.SBAs.ServoSBA;
import org.firstinspires.ftc.teamcode.SBAs.WaitSBA;

@Config
public class Intake extends VelMotor {
    public static double INTAKE_SPEED = 100; // rpm
    public static double MIN_INTAKE_SPEED = 80; // rpm

    public static double SERVO_DOWN_POS = 0.15;
    public static double SERVO_UP_POS = 0.5;
    public static int SERVO_WAIT = 500; // ms

    private Servo servo;

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeMotor", -175);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "intakeServo");
    }

    @Override
    public double getSpeed() {
        return INTAKE_SPEED;
    }

    @Override
    public double getMinSpeed() {
        return MIN_INTAKE_SPEED;
    }

    public void kickUp() {
        servo.setPosition(SERVO_UP_POS);
    }

    public void kickDown() {
        servo.setPosition(SERVO_DOWN_POS);
    }

    public SBA[] kick() {
        return new SBA[] {
                new ServoSBA(servo, SERVO_UP_POS),
                new WaitSBA(SERVO_WAIT),
                new ServoSBA(servo, SERVO_DOWN_POS),
        };
    }
}
