package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.CookedMotor;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.EnhancedMotor;
import org.firstinspires.ftc.teamcode.Utils.PowerAction;

import java.util.Timer;


@Config
public class Intake {

    public static double INTAKE_POWER = 0.5;

    public static double REVERSE_POWER = 0.5;

    public static double WIGGLE_IN_POWER = 0.3;
    public static int WIGGLE_IN_TIME = 1000; // ms
    public static double WIGGLE_OUT_POWER = 0.25;
    public static int WIGGLE_OUT_TIME = 1000; // ms
    public static int WIGGLE_TIMES = 4;

//    public static int IN_WAIT_MS = 300;
//    public static int REVERSE_WAIT_MS = 175;


    // Auto-launch constants
//    public static double SHORT_IN_DIST = 0.25; // revs
//    public static double SHORT_IN_POWER = 0.5;
//    public static double SHORT_OUT_DIST = 0.25; // revs
//    public static double SHORT_OUT_POWER = 0.4;
//    public static double LONG_IN_DIST = 1.5; // revs
//    public static double LONG_IN_POWER = 0.5;
//    public static double LONG_OUT_DIST = 0.5; // revs


    public EnhancedMotor motor;
    public CookedMotor cookedMotor;


    public Intake(HardwareMap hardwareMap) {
        motor = new EnhancedMotor(hardwareMap, "intakeMotor");
        motor.setTicksPerRev(145.1);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cookedMotor = new CookedMotor(motor.motor, 6);
    }


    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public void doStop() {
        // Stop the motor
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public void doTelemetry() {
        Telemetry telemetry = MainBot.shared.telemetry;
        telemetry.addData("intakeMotor Velocity", String.format("%.2f", motor.getRPM())); // getCurrentVelocity()
        telemetry.addData("intakeMotor Power", String.format("%.2f", motor.getPower()));
    }

    public PowerAction getPowerAction(double power) {
        return new PowerAction(motor.motor, power);
    }
}
