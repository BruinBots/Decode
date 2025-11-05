package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;


@Config
public class Intake extends VelMotor {

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_SPEED = 1000; // rpm

    public static double SERVO_DOWN_POS = 0.6;
    public static double SERVO_UP_POS = 0.4;

    public static double REVERSE_POWER = 0.3;

//    public static double INTAKE_STOP_POS = 0.5;
//    public static double INTAKE_IN_POS = 1;
//    public static double INTAKE_REVERSE_POS = 0.35;

//    public static double SERVO_DOWN_POS = 0.6;
//    public static double SERVO_UP_POS = 0.4;

//    private Servo servo;
//    private Servo kickServo;

    public Intake(HardwareMap hardwareMap) {
//        servo = hardwareMap.get(Servo.class, "intakeServo");
//        kickServo = hardwareMap.get(Servo.class, "intakeKickServo");
        super(hardwareMap, "intakeMotor", "kickServo", 145.1);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void doTelemetry() {
//        MainBot.shared.telemetry.addData("intakeServo", servo.getPosition());
//        MainBot.shared.telemetry.addData("intakeKickServo", kickServo.getPosition());
//    }

//    public void spinUp() {
//        servo.setPosition(INTAKE_IN_POS);
//    }
//
//    public void reverse() {
//        servo.setPosition(INTAKE_REVERSE_POS);
//    }
//
//    public void stop() {
//        servo.setPosition(INTAKE_STOP_POS);
//    }

//    public void kickUp() {
//        kickServo.setPosition(SERVO_UP_POS);
//    }
//
//    public void kickDown() {
//        kickServo.setPosition(SERVO_DOWN_POS);
//    }
//
//    public ServoAction getServoAction(double pos) {
//        return new ServoAction(servo, pos);
//    }
}
