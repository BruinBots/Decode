package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.EnhancedMotor;

@Config
public class Lifter {
    public EnhancedMotor motor;

    private double ticksPerRev = 5281.1;

    public static int OFF_POSITION = 0;
    public static int ON_POSITION = -650;

    public static double ON_POWER = 0.5;
    public static double OFF_POWER = 0.2;

    public Lifter(HardwareMap hardwareMap) {
        motor = new EnhancedMotor(hardwareMap, "lifter");
        motor.setTicksPerRev(ticksPerRev);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void doTelemetry() {
        MainBot.shared.telemetry.addData("Lifter Motor", motor.motor.getCurrentPosition()+"/"+motor.motor.getTargetPosition());
    }

    public void doLift() {
        motor.motor.setTargetPosition(ON_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(ON_POWER);
    }

    public void unLift() {
        motor.motor.setTargetPosition(OFF_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(OFF_POWER);
    }
}
