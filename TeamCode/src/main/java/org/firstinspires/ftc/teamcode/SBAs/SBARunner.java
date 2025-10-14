package org.firstinspires.ftc.teamcode.SBAs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SBARunner {
    public ArrayList<SBA> curSBAs;
    public int curIdx;
    public int curInitIdx;

    public void runSBAs(SBA... sbas) {
        runSBAs(sbas, false);
    }

    public void runSBAs(SBA[] sbas, boolean overwrite) {
        if (overwrite || curSBAs == null) {
            curSBAs = new ArrayList<>(Arrays.asList(sbas));
            curIdx = 0;
            curInitIdx = -1;
        } else {
            curSBAs.addAll(Arrays.asList(sbas));
        }
    }

    public void stop() {
        runSBAs(new SBA[]{}, true); // overwrite with empty list
    }

    public boolean isBusy() {
        return curSBAs.size() > 0;
    }

    public void loop() {
        // Ensure we're currently running an SBA list
        if (curSBAs.size() == 0 || curSBAs.size() < curIdx - 1) {
            return;
        }
        SBA sba = curSBAs.get(curIdx);
        if (curInitIdx < curIdx) {
            sba.preInit();
            if (!sba.sanity()) {
                stop();
            } // Quit if sanity check fails
            sba.init();
            curInitIdx = curIdx;
        }
        sba.loop();
        if (!sba.isBusy()) { // move onto next SBA if done with current one
            curIdx++;
            if (curIdx > curSBAs.size() - 1) {
                stop();
            } // End SBA list if curIdx gets past the list limit
        }
    }

    // TODO: SBA Stubs down here
    /*
    Example:
    public void runArmUp() {
        runSBAs(new MotorSBA(bot.armMotor, Arm.POWER, Arm.TOP_POS));
    }
     */
}
