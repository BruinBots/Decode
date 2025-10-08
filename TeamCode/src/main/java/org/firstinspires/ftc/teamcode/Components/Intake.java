package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake extends VelMotor {
    public static double INTAKE_SPEED = 300; // rpm
    public static double MIN_INTAKE_SPEED = 250; // rpm

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, "test", -175);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
