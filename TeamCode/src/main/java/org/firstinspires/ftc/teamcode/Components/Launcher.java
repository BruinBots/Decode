package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Launcher extends VelMotor {
    public static double LAUNCH_SPEED = 300; // rpm
    public static double MIN_LAUNCH_SPEED = 250; // rpm

    public Launcher(HardwareMap hardwareMap) {
        super(hardwareMap, "launchMotor", 27);
    }

    @Override
    public double getSpeed() {
        return LAUNCH_SPEED;
    }

    @Override
    public double getMinSpeed() {
        return MIN_LAUNCH_SPEED;
    }
}
