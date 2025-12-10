package org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

public class ShooterKickCommand extends CommandBase {

    private final Shooter m_shooter;
    private Timer timer;

    private enum KickState {
        INIT,
        UP,
        DOWN,
        DONE
    }
    private KickState state = KickState.INIT;

    public ShooterKickCommand(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.resetTimer();
    }

    @Override
    public void execute() {
        switch (state) {
            case INIT:
                m_shooter.kickUp();
                state = KickState.UP;
                timer.resetTimer();
                break;
            case UP:
                if (timer.getElapsedTimeSeconds() > Shooter.KICK_UP_DURATION) {
                    m_shooter.kickDown();
                    state = KickState.DOWN;
                    timer.resetTimer();
                }
                break;
            case DOWN:
                if (timer.getElapsedTimeSeconds() > Shooter.KICK_DOWN_DURATION) {
                    state = KickState.DONE;
                    timer.resetTimer();
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == KickState.DONE;
    }
}
