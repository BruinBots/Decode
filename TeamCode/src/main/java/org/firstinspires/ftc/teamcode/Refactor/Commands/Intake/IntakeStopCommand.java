package org.firstinspires.ftc.teamcode.Refactor.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;

import java.util.Set;

public class IntakeStopCommand extends CommandBase {

    private final Intake m_intake;

    public IntakeStopCommand(Intake intake) {
        this.m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}