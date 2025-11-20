package org.firstinspires.ftc.teamcode.Utils;

import android.os.health.SystemHealthManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class CookedMotor {
    /*
    Motor overcurrent protection
    When the motor exceeds maxCurrent (typical GoBilda 5202/3/4 series are rated to 9A),
    the motor will briefly stop and the linked gamepad will rumble.
     */
    private DcMotorEx motor;
    private double maxCurrent = 5.0; // Amps

    private long lastNotCookedTime;
    public static double COOK_DURATION = 1000;

    public CookedMotor(DcMotorEx motor, double maxCurrent) {
        this.motor = motor;
        this.maxCurrent = maxCurrent;
        lastNotCookedTime = System.currentTimeMillis();
    }

    public boolean isCooked() {
        boolean cooked = motor.getCurrent(CurrentUnit.AMPS) > maxCurrent;
        if (!cooked) { lastNotCookedTime = System.currentTimeMillis(); }
        return cooked;
    }

    public void loop() {
        loop(null);
    }

    public void loop(Gamepad gamepad) {
        if (isCooked() && System.currentTimeMillis() - lastNotCookedTime > COOK_DURATION) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad != null) {
                if (!gamepad.isRumbling()) {
                    gamepad.rumble(500);
                }
            }
        }
    }
}
