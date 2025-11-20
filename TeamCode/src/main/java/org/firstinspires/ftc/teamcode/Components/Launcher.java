package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.CookedMotor;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.AccelWaitAction;
import org.firstinspires.ftc.teamcode.Utils.EnhancedMotor;
import org.firstinspires.ftc.teamcode.Utils.PowerAction;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;
import org.firstinspires.ftc.teamcode.Utils.WaitAction;

@Config
public class Launcher { // extends VelMotor {

    /*
    Launch powers:
    (distance, power)
    (117, 0.9)
    (64, 0.74)
    (35, 0.74)
    Near: 0.75
    Far: 0.9
     */

    public static double LAUNCH_POWER = QuickAimBot.CLOSE_POWER;
    public static double ACTIVE_SPEED = 50; // rpm
    public static double MAX_LAUNCH_ACCEL = 0.01; // rpm/s^2

    public static double REVERSE_POWER = 0.075;

    public static double SERVO_DOWN_POS = 0.25;
    public static double SERVO_UP_POS = 0.42;

    // time to wait after spin up to initiate accel check
    // give the motor a little time to accelerate
    public static int LAUNCH_WAIT_MS = 750;

    public static int SERVO_WAIT_MS = 1250;
    public static int POST_LAUNCH_WAIT_MS = 500;

    public CookedMotor cookedMotor;

    public EnhancedMotor motor;
    public Servo servo;

    public Launcher(HardwareMap hardwareMap) {
        motor = new EnhancedMotor(hardwareMap, "launchMotor");
        motor.setTicksPerRev(28);
        servo = hardwareMap.get(Servo.class, "kickServo");
        cookedMotor = new CookedMotor(motor.motor, 6);
    }

    public boolean isActive() {
        return motor.getRPM() > ACTIVE_SPEED || motor.getPower() > 0;
    }

    public void setServo(double position) {
        servo.setPosition(position);
    }

    public void spinUp(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public void doTelemetry() {
        MainBot.shared.telemetry.addData("Launcher Power", motor.getPower());
        MainBot.shared.telemetry.addData("Launcher Speed", motor.getRPM()); // getCurrentVelocity()
        MainBot.shared.telemetry.addData("Launcher Acceleration", motor.getAcceleration()); // getCurrentAcceleration()
    }

    public void doStop() {
        spinUp(0);
    }

    public Action kickAction() {
        return new SequentialAction(
            new ServoAction(servo, SERVO_UP_POS),
            new WaitAction(SERVO_WAIT_MS),
            new ServoAction(servo, SERVO_DOWN_POS)
        );
    }

    public PowerAction getPowerAction(double power) {
        return new PowerAction(motor.motor, power);
    }

    public AccelWaitAction getAccelWaitAction(double maxAccel) {
        return new AccelWaitAction(motor, maxAccel);
    }
}
