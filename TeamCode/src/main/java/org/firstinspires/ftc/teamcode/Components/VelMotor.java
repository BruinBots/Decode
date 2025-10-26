package org.firstinspires.ftc.teamcode.Components;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainBot;

public class VelMotor {
    public DcMotorEx motor;
    public Servo servo;
    private String motorName;
    private String servoName;

    private double ticksPerRev;

    public VelMotor(HardwareMap hardwareMap, String motorName, String servoName, double ticksPerRev) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo = hardwareMap.get(Servo.class, servoName);

        this.motorName = motorName;
        this.servoName = servoName;
        this.ticksPerRev = ticksPerRev;
    }

    public double getSpeed() { return 0.0; }
    public double getMinSpeed() { return 0.0; }

    public double getUpPosition() { return 0.0; }
    public double getDownPosition() { return 0.0; }

    public void spinUp() {
        // Set motor target velocity to the desired speed
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(getSpeed()*ticksPerRev/60.0);
    }

    public boolean isReady() {
        // Returns if the motor is at the target speed
        return getCurrentVelocity() >= getMinSpeed();
    }

    public void doStop() {
        // Stop the motor
        motor.setVelocity(0);
        motor.setPower(0);
    }

    public void servoUp() {
        servo.setPosition(getUpPosition());
    }

    public void servoDown() {
        servo.setPosition(getDownPosition());
    }

    public boolean isStopped() {
        return getCurrentVelocity() < 30;
    }

    public double getCurrentVelocity() {
        return motor.getVelocity()*60.0/ticksPerRev;
    }

    public void doTelemetry() {
        Telemetry telemetry = MainBot.shared.telemetry;
        telemetry.addData(motorName + " Velocity", String.format("%.2f/%.2f", getCurrentVelocity(), getSpeed()*ticksPerRev/60.0));
        telemetry.addData(motorName + " Power", String.format("%.2f", motor.getPower()));
    }

    class SpinUpAction implements Action {
        private VelMotor motor;
        private boolean firstLoop = true;

        private SpinUpAction(VelMotor motor) {
            this.motor = motor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                motor.spinUp();
                firstLoop = false;
                return true;
            }
            return motor.isReady();
        }
    }

    public SpinUpAction getSpinUpAction() {
        return new SpinUpAction(this);
    }

    class StopAction implements Action {
        private VelMotor motor;
        private boolean firstLoop = true;

        private StopAction(VelMotor motor) {
            this.motor = motor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                motor.doStop();
                firstLoop = false;
                return true;
            }
            return motor.isStopped();
        }
    }

    public StopAction getStopAction() {
        return new StopAction(this);
    }

    class ServoAction implements Action {
        private VelMotor motor;
        private boolean isUp;

        private ServoAction(VelMotor motor, boolean isUp) {
            this.motor = motor;
            this.isUp = isUp;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isUp) {
                motor.servoUp();
            } else {
                motor.servoDown();
            }
            return false;
        }
    }

    public ServoAction servoUpAction() {
        return new ServoAction(this, true);
    }

    public ServoAction servoDownAction() {
        return new ServoAction(this, false);
    }
}
