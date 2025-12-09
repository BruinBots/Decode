package org.firstinspires.ftc.teamcode.Refactor;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;

@Configurable
public class Jimmy {

    public static Jimmy shared;

    public final Intake intake;

    public Jimmy(HardwareMap hMap) {
        intake = new Intake(hMap, "intakeMotor");
    }
}
