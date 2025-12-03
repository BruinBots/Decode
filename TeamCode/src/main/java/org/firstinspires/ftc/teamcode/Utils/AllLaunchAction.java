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
    private double launchVel;
    private int maxLaunches;
    private int numLaunches = 0;

    public AllLaunchAction(AimBot aimBot) {
        this.aimBot = aimBot;
        maxLaunches = 3;
    }

    public AllLaunchAction(AimBot aimBot, int maxLaunches) {
        this.aimBot = aimBot;
        this.maxLaunches = maxLaunches;
    }

    public AllLaunchAction(double launchVel) {
        this.launchVel = launchVel;
        this.maxLaunches = 3;
    }

    public AllLaunchAction(double launchVel, int maxLaunches) {
        this.launchVel = launchVel;
        this.maxLaunches = maxLaunches;
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
                MainBot.shared.launcher.spinUp(0); // stop launcher, done with launches
                return false;
            }
        }
        if (MainBot.shared.launcher.artifactPresent) {
            lastArtifactTime = curTime;
        }
        if (launchAction == null && MainBot.shared.launcher.artifactPresent) {
            double vel;
            if (aimBot != null) {
                vel = aimBot.getLaunchVel();
            } else {
                vel = launchVel;
            }
            if (numLaunches == maxLaunches - 1) { // last launch}
                launchAction = MainBot.shared.singleLaunchActionNoPreload(vel);
//                launchAction = MainBot.shared.singleLaunchAction(power);
            } else {
                launchAction = MainBot.shared.singleLaunchAction(vel);
            }
        } else if (launchAction != null) {
            if (!launchAction.run(telemetryPacket)) {
                // launch done, continue
                launchAction = null;
                lastArtifactTime = curTime;
                numLaunches ++;
                if (numLaunches >= maxLaunches) {
                    MainBot.shared.launcher.spinUp(0); // stop launcher, done with launches
                    return false; // done
                }
            }
        }

        telemetryPacket.addLine("lastArtifactTime = "+lastArtifactTime);
        telemetryPacket.addLine("curTime = "+curTime);
        return true;
    }
}
