package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.teamcode.SBAs.ServoSBA;
import org.firstinspires.ftc.teamcode.SBAs.WaitSBA;

@Config
public class Launcher extends VelMotor {
    public static double LAUNCH_SPEED = 300; // rpm
    public static double MIN_LAUNCH_SPEED = 250; // rpm

    public static double SERVO_DOWN_POS = 0;
    public static double SERVO_UP_POS = 0;
    public static int SERVO_WAIT = 1000; // ms

    private Servo servo;

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", 27);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "kickServo");
    }

    @Override
    public double getSpeed() {
        return LAUNCH_SPEED;
    }

    @Override
    public double getMinSpeed() {
        return MIN_LAUNCH_SPEED;
    }

    public SBA[] kick() {
        return new SBA[] {
                new ServoSBA(servo, SERVO_UP_POS),
                new WaitSBA(SERVO_WAIT),
                new ServoSBA(servo, SERVO_DOWN_POS),
        };
    }
}
