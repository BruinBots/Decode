package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MainBot;

public class ServoAction implements Action {
    private Servo servo;
    private double position;

    public ServoAction(Servo servo, double position) {
        this.servo = servo;
        this.position = position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        servo.setPosition(position);
        telemetryPacket.addLine("ServoAction "+position);
        return false;
    }
}