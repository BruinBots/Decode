package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake extends VelMotor {
    public static double INTAKE_SPEED = 2.0; // revs/sec
    public static double MIN_INTAKE_SPEED = 1.5; // revs/sec

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeMotor");
    }

    @Override
    public double getSpeed() {
        return INTAKE_SPEED;
    }

    @Override
    public double getMinSpeed() {
        return MIN_INTAKE_SPEED;
    }
}
