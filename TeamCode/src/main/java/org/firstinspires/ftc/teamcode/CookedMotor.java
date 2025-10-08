package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CookedMotor {
    private DcMotorEx motor;
    private double maxCurrent = 5.0; // Amps

    public CookedMotor(DcMotorEx motor, double maxCurrent) {
        this.motor = motor;
        this.maxCurrent = maxCurrent;
    }

    public boolean isCooked() {
        return motor.getCurrent(CurrentUnit.AMPS) > maxCurrent;
    }

    public void loop() {
        if (isCooked()) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
