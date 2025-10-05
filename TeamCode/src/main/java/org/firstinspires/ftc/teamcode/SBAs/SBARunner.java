package org.firstinspires.ftc.teamcode.SBAs;

public class SBARunner {
    public SBA[] curSBAs;
    public int curIdx;

    public void runSBAs(SBA... sbas) {
        curSBAs = sbas;
        curIdx = 0;
    }

    public void stop() {
        runSBAs(new SBA[]{});
    }

    public void loop() {
        // Ensure we're currently running an SBA list
        if (curSBAs.length == 0 || curSBAs.length < curIdx - 1) {
            return;
        }
        SBA sba = curSBAs[curIdx];
        sba.preInit();
        if (!sba.sanity()) {
            stop();
        } // Quit if sanity check fails
        sba.init();
        sba.loop();
        if (!sba.isBusy()) { // move onto next SBA if done with current one
            curIdx++;
            if (curIdx > curSBAs.length - 1) {
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
