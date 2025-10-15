package org.firstinspires.ftc.teamcode.SBAs;

public interface SBA {

    void preInit();
    boolean sanity();
    void init();
    void loop();
    boolean isBusy();

    public default SBAAction action() {
        return new SBAAction(this);
    }
}