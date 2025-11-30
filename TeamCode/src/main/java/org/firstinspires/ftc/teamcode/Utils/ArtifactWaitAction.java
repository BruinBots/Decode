package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;

public class ArtifactWaitAction implements Action {
    private int maxWait;
    private long startTime = 0;

    public ArtifactWaitAction(int maxWait) {
        this.maxWait = maxWait;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
        }
        long curTime = System.currentTimeMillis();
        if (MainBot.shared.launcher.artifactPresent) {
            return false; // done, artifact present
        }
        if (curTime - startTime > maxWait) {
            return false; // done, waited too long
        }
        return true; // keep waiting
    }
}
