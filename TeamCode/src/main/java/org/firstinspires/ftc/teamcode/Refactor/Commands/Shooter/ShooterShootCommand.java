package org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

public class ShooterShootCommand extends CommandBase {

    private final Shooter m_shooter;
    private final double m_speed;
    private final int m_num_shots;

    public ShooterShootCommand(Shooter shooter, double speed, int numShots) {
        m_shooter = shooter;
        m_speed = speed;
        m_num_shots = numShots;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setSpeed(m_speed);
        m_shooter.fireShots(m_num_shots);
    }

    @Override
    public boolean isFinished() {
        return !m_shooter.isBusy();
    }
}
