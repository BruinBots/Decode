package org.firstinspires.ftc.teamcode.SBAs;

public interface SBA {

    void preInit();
    boolean sanity();
    void init();
    void loop();
    boolean isBusy();
}