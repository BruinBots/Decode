package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CookedMotor;
import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.teamcode.Utils.EnhancedMotor;
import org.firstinspires.ftc.teamcode.Utils.PowerAction;


@Config
public class Intake {

    public static double INTAKE_POWER = 0.5;

    public static double REVERSE_POWER = 0.4;

    public static int IN_WAIT_MS = 300;
    public static int REVERSE_WAIT_MS = 250;

    public EnhancedMotor motor;
    public CookedMotor cookedMotor;


    public Intake(HardwareMap hardwareMap) {
        motor = new EnhancedMotor(hardwareMap, "intakeMotor");
        motor.setTicksPerRev(145.1);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
