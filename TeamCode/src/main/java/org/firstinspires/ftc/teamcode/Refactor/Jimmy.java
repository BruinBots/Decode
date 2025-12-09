package org.firstinspires.ftc.teamcode.Refactor;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;

@Configurable
public class Jimmy {

    public static Jimmy shared;

    private LynxModule controlHub;
    private LynxModule expansionHub;

    public final Intake intake;

    public Jimmy(HardwareMap hMap) {
        for (LynxModule mod: hMap.getAll(LynxModule.class)) {
            if (mod.isParent()) {
                controlHub = mod;
            } else {
                expansionHub = mod;
            }
        }

        intake = new Intake(hMap, "intakeMotor");
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }
}
