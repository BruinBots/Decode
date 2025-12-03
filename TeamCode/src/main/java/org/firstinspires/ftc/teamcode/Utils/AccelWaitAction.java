package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;

public class AccelWaitAction implements Action {
    private EnhancedMotor motor;
    private double targetVel;
    private double maxAccel;

    public AccelWaitAction(EnhancedMotor motor, double targetVel, double maxAccel) {
        this.motor = motor;
        this.targetVel = targetVel;
        this.maxAccel = maxAccel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        motor.setTargetVelocity(targetVel);
        motor.updateVelocityPID();
        double accel = motor.getAcceleration();
        telemetryPacket.addLine("AccelWaitAction "+accel+"("+maxAccel+"), v="+targetVel);
        return accel > maxAccel;
    }
}