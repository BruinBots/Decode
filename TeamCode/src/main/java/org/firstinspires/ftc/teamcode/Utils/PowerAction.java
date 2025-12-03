package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MainBot;

public class PowerAction implements Action {
    private DcMotorEx motor;
    private EnhancedMotor emotor;
    private double power;

    public PowerAction(DcMotorEx motor, double power) {
        this.motor = motor;
        this.power = power;
    }

    public PowerAction(EnhancedMotor emotor, double power) {
        this.emotor = emotor;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        } else {
            emotor.setTargetVelocity(0);
            emotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            emotor.setPower(power);
        }
        telemetryPacket.addLine("PowerAction "+power);
        return false; // end instantly
    }
}