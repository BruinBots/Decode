package org.firstinspires.ftc.teamcode.Refactor;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

@Configurable
public class Jimmy {

    public static Jimmy shared;

    private LynxModule controlHub;
    private LynxModule expansionHub;

    private final Intake intake;
    private final Shooter shooter;

    public enum Alliance {
        BLUE,
        RED,
        UNKNOWN
    }
    public Alliance alliance;

    public Jimmy(HardwareMap hMap) {
        for (LynxModule mod: hMap.getAll(LynxModule.class)) {
            if (mod.isParent()) {
                controlHub = mod;
            } else {
                expansionHub = mod;
            }
        }

        intake = new Intake(hMap, "intakeMotor");
        shooter = new Shooter(hMap, "launchMotor");
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }
}
