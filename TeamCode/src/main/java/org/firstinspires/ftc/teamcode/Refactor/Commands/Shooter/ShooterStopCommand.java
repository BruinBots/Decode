package org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

public class ShooterStopCommand extends CommandBase {

    private final Shooter m_shooter;
    private final boolean m_brake;

    public ShooterStopCommand(Shooter shooter) {
        m_shooter = shooter;
        m_brake = false;

        addRequirements(shooter);
    }

    public ShooterStopCommand(Shooter shooter, boolean brake) {
        m_shooter = shooter;
        m_brake = brake;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.stop(m_brake);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
