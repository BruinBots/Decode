package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.SBAs.SBA;

public class VelMotor {
    private DcMotorEx motor;
    private String motorName;

    public VelMotor(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorName = motorName;
    }

    public double getSpeed() { return 0.0; }
    public double getMinSpeed() { return 0.0; }

    public void spinUp() {
        // Set motor target velocity to the desired speed
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(getSpeed() *360.0, AngleUnit.DEGREES);
    }

    public boolean isReady() {
        // Returns if the motor is at the target speed
        double curVel = motor.getVelocity(AngleUnit.DEGREES);
        return curVel >= getMinSpeed();
    }

    public void doStop() {
        // Stop the motor
        motor.setVelocity(0);
        motor.setPower(0);
    }

    public boolean isStopped() {
        return motor.getVelocity() < 0.5;
    }

    public void doTelemetry() {
        Telemetry telemetry = MainBot.shared.telemetry;
        telemetry.addData(motorName + " Velocity", motor.getVelocity(AngleUnit.DEGREES));
        telemetry.addData(motorName + " Power", motor.getPower());
    }

    // SBA Integration
    public class VelMotorSBA implements SBA {
        private VelMotor vMotor;
        private boolean spinUp;

        public VelMotorSBA(VelMotor vMotor, boolean spinUp) {
            this.vMotor = vMotor;
            this.spinUp = spinUp;
        }

        @Override
        public boolean sanity() {
            return true;
        }

        @Override
        public void init() {
            if (spinUp) {
                vMotor.spinUp();
            } else {
                vMotor.doStop();
            }
        }

        @Override
        public void loop() {

        }

        @Override
        public boolean isBusy() {
            if (spinUp) {
                return !vMotor.isReady();
            } else {
                return !vMotor.isStopped();
            }
        }
    }

    public SBA getSpinUpSBA() {
        return new VelMotorSBA(this, true);
    }

    public SBA getStopSBA() {
        return new VelMotorSBA(this, false);
    }
}
