package org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

public class ShooterStopAction extends CommandBase {

    private final Shooter m_shooter;
    private final boolean m_brake;

    public ShooterStopAction(Shooter shooter) {
        m_shooter = shooter;
        m_brake = false;

        addRequirements(shooter);
    }

    public ShooterStopAction(Shooter shooter, boolean brake) {
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
