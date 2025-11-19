package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;

public class WaitAction implements Action {
    private int waitMs;
    private long startTime = 0;

    public WaitAction(int waitMs) {
        this.waitMs = waitMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
            telemetryPacket.addLine("WaitAction "+waitMs+"ms");
            return true;
        }
        return System.currentTimeMillis() <= startTime + waitMs;
    }
}
