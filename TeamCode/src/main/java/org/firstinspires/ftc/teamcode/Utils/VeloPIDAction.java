package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class VeloPIDAction implements Action {
    private EnhancedMotor motor;
    private double targetVelocity; // in ticks/sec or RPM depending on your encoder
    private double kP = 0.001; // tune these
    private double kI = 0.0;
    private double kD = 0.0;

    private double integral = 0;
    private double lastError = 0;
    public VeloPIDAction(EnhancedMotor motor, double targetVelocity) {
        this.motor = motor;
        this.targetVelocity = targetVelocity;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double currentVelocity = motor.getRPM();

        double error = targetVelocity - currentVelocity;
        integral += error;
        double derivative = error - lastError;

        double output = kP * error + kI * integral + kD * derivative;

        // clamp output between -1 and 1
        output = Math.max(-1, Math.min(1, output));

        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(output);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(output);
        }

        lastError = error;

        telemetryPacket.addLine("VeloPIDAction -> "+output);

        return false; // ends instantly; for continuous control, wrap in a loop or repeated Action
    }
}