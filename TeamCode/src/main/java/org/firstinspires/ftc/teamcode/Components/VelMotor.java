package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CookedMotor;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;

public class VelMotor {
    public DcMotorEx motor;
    public Servo servo;
    public CookedMotor cookedMotor;

    private String motorName;
    private String servoName;

    private double ticksPerRev;
    private double targetVelocity;
    private PIDController pidController;

    public VelMotor(HardwareMap hardwareMap, String motorName, String servoName, double ticksPerRev) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cookedMotor = new CookedMotor(motor, 6);

        servo = hardwareMap.get(Servo.class, servoName);

        this.motorName = motorName;
        this.servoName = servoName;
        this.ticksPerRev = ticksPerRev;

        pidController = new PIDController(0,0,0);
    }

    public double getVelKp() { return 0.0; }

    public PIDCoefficients getVelPIDCoefficients() { return new PIDCoefficients(0,0,0); }

    public void spinUp(double power) {
        // Set motor target velocity to the desired speed
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        PIDCoefficients coeffs = getVelPIDCoefficients();
        pidController.setPID(coeffs);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(pidController.calculate(getCurrentVelocity(), velocity));
    }

    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public boolean isAtSpeed(double speed) {
        // Returns if the motor is at the target speed
        return getCurrentVelocity() >= speed;
    }

    public void doStop() {
        // Stop the motor
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public void setServo(double position) {
        servo.setPosition(position);
    }

    public boolean isStopped() {
        return getCurrentVelocity() < 30;
    }

    public double getCurrentVelocity() {
        return motor.getVelocity() * 60.0 / ticksPerRev;
    }

    public void doTelemetry() {
        Telemetry telemetry = MainBot.shared.telemetry;
        telemetry.addData(motorName + " Velocity", String.format("%.2f", getCurrentVelocity()));
        telemetry.addData(motorName + " Power", String.format("%.2f", motor.getPower()));
    }

    class SpinUpAction implements Action {
        private VelMotor motor;
        private double power;
        private double speed;
        private boolean firstLoop = true;

        private SpinUpAction(VelMotor motor, double power, double speed) {
            this.motor = motor;
            this.power = power;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                // Velocity P-controller
                double curVel = motor.getCurrentVelocity();
                double velError = speed - curVel;
                double adjustedPower = power + (motor.getVelKp() * velError);
                if (adjustedPower > 1.0) {
                    motor.spinUp(1);
                } else {
                    motor.spinUp(adjustedPower);
                }
                telemetryPacket.addLine("curVel="+curVel);
                telemetryPacket.addLine("adjustedPower="+adjustedPower);
//                motor.setTargetVelocity(speed);
                firstLoop = false;
                return true;
            }
            if (speed > 0) {
                return !motor.isAtSpeed(speed);
            } else {
                return !motor.isStopped();
            }
        }
    }

    class PowerAction implements Action {
        private VelMotor motor;
        private double power;

        public PowerAction(VelMotor motor, double power) {
            this.motor = motor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.motor.setPower(power);
            return false; // end instantly
        }
    }

    public PowerAction getPowerAction(double power) {
        return new PowerAction(this, power);
    }

    public SpinUpAction getSpinUpAction(double power, double speed) {
        return new SpinUpAction(this, power, speed);
    }

    public ServoAction getServoAction(double position) {
        return new ServoAction(servo, position);
    }
}
