package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MainTeleOp;

@Config
public class ArtifactShakeAction implements Action {
    private int maxWait;
    private long startTime = 0;

    public static double SHAKE_SEGMENT_POWER = 0.5;
    public static double SHAKE_SEGMENT_DURATION = 500; // ms
    public static double SHAKE_EXTRA_TIME = 750; // ms

    private double lastSegmentTime = 0;
    private double lastSegmentDir = -1;

    public ArtifactShakeAction(int maxWait) {
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
        if (curTime - startTime > (maxWait + SHAKE_EXTRA_TIME)) {
            // too long, just exit
            MainBot.shared.moveBotMecanum(0, 0, 0, 1);
            return false;
        }
        if (curTime - startTime > maxWait) {
            // shake
            if (curTime - lastSegmentTime > SHAKE_SEGMENT_DURATION) {
                // done with last shake, start new one
                lastSegmentDir *= -1;
                lastSegmentTime = curTime;
            }
            MainBot.shared.moveBotMecanum(lastSegmentDir*SHAKE_SEGMENT_POWER, 0, 0, 1);
        }
        return true;
    }
}
