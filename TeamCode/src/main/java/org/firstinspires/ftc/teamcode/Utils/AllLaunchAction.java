package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.MainBot;

@Config
public class AllLaunchAction implements Action {

    public static int WAIT_BETWEEN_DURATION = 1250;

    private long lastArtifactTime;
    private Action launchAction = null;
    private boolean firstLoop = true;
    private AimBot aimBot;

    public AllLaunchAction(AimBot aimBot) {
//        lastArtifactTime = System.currentTimeMillis();
        this.aimBot = aimBot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (firstLoop) {
            lastArtifactTime = System.currentTimeMillis();
            firstLoop = false;
        }
        long curTime = System.currentTimeMillis();
        if (launchAction == null) {
            if (curTime - lastArtifactTime > WAIT_BETWEEN_DURATION) {
                // nothing more to launch
                return false;
            }
        }
        if (MainBot.shared.launcher.artifactPresent) {
            lastArtifactTime = curTime;
        }
        if (launchAction == null && MainBot.shared.launcher.artifactPresent) {
            launchAction = MainBot.shared.singleLaunchAction(aimBot.getLaunchPower());
        } else if (launchAction != null) {
            if (!launchAction.run(telemetryPacket)) {
                // launch done, continue
                launchAction = null;
                lastArtifactTime = curTime;
            }
        }

        telemetryPacket.addLine("lastArtifactTime = "+lastArtifactTime);
        telemetryPacket.addLine("curTime = "+curTime);
        return true;
    }
}
