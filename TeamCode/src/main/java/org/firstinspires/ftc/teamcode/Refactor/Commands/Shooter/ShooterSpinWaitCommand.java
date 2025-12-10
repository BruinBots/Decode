package org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

public class ShooterSpinWaitCommand extends CommandBase {

    private final Shooter m_shooter;
    private final double m_speed;

    public ShooterSpinWaitCommand(Shooter shooter, double speed) {
        m_shooter = shooter;
        m_speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooter.getSpeed() - m_speed) < Shooter.SPEED_TOLERANCE;
    }
}
