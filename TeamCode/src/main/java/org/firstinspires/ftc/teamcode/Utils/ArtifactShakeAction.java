package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.MainTeleOp;

@Config
public class ArtifactShakeAction implements Action {
    private int preWait;
    private int maxWait;
    private Action tryAction;

    private long preStartTime = 0;
    private long startTime = 0;

    public static int PRE_WAIT = 750;
    public static int MAX_WAIT = 1500; // ms

    public ArtifactShakeAction(int preWait, int maxWait, Action tryAction) {
        this.preWait = preWait;
        this.maxWait = maxWait;
        this.tryAction = tryAction;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (preStartTime == 0) {
            preStartTime = System.currentTimeMillis();
        }
        long curTime = System.currentTimeMillis();
        if (!MainBot.shared.launcher.artifactPresent) {
            // artifact registered empty, now we can continue
            startTime = curTime;
        }
        if (curTime - preStartTime > preWait) {
            return false; // waited too long for launcher to empty
        }
        if (startTime != 0) {
            if (MainBot.shared.launcher.artifactPresent) {
                return false; // done, artifact present
            }
            if (curTime - startTime > maxWait) {
                // too long, just exit
                return false;
            }
            tryAction.run(telemetryPacket);
            telemetryPacket.addLine("ArtifactShakeAction " + (curTime - startTime) + "/" + maxWait);
        }
        return true;
    }
}
