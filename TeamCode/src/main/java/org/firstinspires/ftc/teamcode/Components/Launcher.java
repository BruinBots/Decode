package org.firstinspires.ftc.teamcode.Components;

import android.app.Notification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CookedMotor;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;

@Config
public class Launcher { // extends VelMotor {

    /*
    (117, 0.9)
    (64, 0.74)
    (35, 0.74)
    Near: 0.75
    Far: 0.9
     vbb
     */

    public static double LAUNCH_POWER = 0.7;
    public static double LAUNCH_SPEED = 3800; // rpm

    public static double REVERSE_POWER = 0.075;

    public static double SERVO_DOWN_POS = 0.25;
    public static double SERVO_UP_POS = 0.35;

    public static int LAUNCH_WAIT_MS = 3000;

    public static int SERVO_WAIT_MS = 1250;
    public static int POST_LAUNCH_WAIT_MS = 500;

    public static double TICKS_PER_REV = 28;

    public CookedMotor cookedMotor;

    private DcMotorEx motor;
    private Servo servo;

    public static double ACCEL_INTERVAL = 500; // ms between accel reads

    private double accel;
    private double lastVel;
    private double lastVelTime;

    public Launcher(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        servo = hardwareMap.get(Servo.class, "kickServo");
        cookedMotor = new CookedMotor(motor, 6);

        lastVelTime = System.currentTimeMillis();
        lastVel = 0;
        accel = 0;
    }

    public class AccelerationAction implements Action {
        private Launcher launcher;
        private double power;
        private double maxAccel;
        private boolean firstLoop = true;
        private double startTime;
        public AccelerationAction(Launcher launcher, double power, double maxAccel) {
            this.launcher = launcher;
            this.power = power;
            this.maxAccel = maxAccel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                launcher.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launcher.motor.setPower(power);

                firstLoop = false;
            }
            double accel = launcher.getCurrentAcceleration();
            return accel > maxAccel;
        }
    }

    public class SpinUpAction implements Action {
        private Launcher launcher;
        private double power;
        private double speed;

        public SpinUpAction(Launcher launcher, double power, double speed) {
            this.launcher = launcher;
            this.power = power;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Move the motor
            launcher.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.motor.setPower(power);

            // Check if within bounds
            double error = Math.abs(launcher.getCurrentVelocity() - speed);
            if (error < 30) {
                launcher.motor.setPower(0);
                return false; // stop
            }
            return true; // keep going
        }
    }

    public Action getSpinUpAction(double power, double speed) {
        return getSpinUpAction(power, speed);
    }

    public double getCurrentVelocity() {
        double curVel = motor.getVelocity() * 60.0 / TICKS_PER_REV;
        double curTime = System.currentTimeMillis();

        if (curTime - lastVelTime > ACCEL_INTERVAL) {
            double dv = curVel - lastVel;
            double dt = curTime - lastVelTime;
            accel = dv / dt;

            lastVel = curVel;
            lastVelTime = curTime;
        }

        return curVel;
    }

    public double getCurrentAcceleration() {
        return accel;
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
        MainBot.shared.telemetry.addData("Launcher Speed", getCurrentVelocity());
        MainBot.shared.telemetry.addData("Launcher Acceleration", getCurrentAcceleration());
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

    class PowerAction implements Action {
        private DcMotorEx motor;
        private double power;

        public PowerAction(DcMotorEx motor, double power) {
            this.motor = motor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
            return false; // end instantly
        }
    }

    public Launcher.PowerAction getPowerAction(double power) {
        return new Launcher.PowerAction(motor, power);
    }
}
