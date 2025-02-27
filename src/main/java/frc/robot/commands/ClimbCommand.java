/*
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    private ClimbSubsystem m_subsystem;
    private boolean reverse;

    public ClimbCommand(ClimbSubsystem m_ClimbSubsystem, boolean reverse)
    {
        this.m_subsystem = m_subsystem;
        this.reverse = reverse;
        addRequirements(m_subsystem);
    }

    public void initialize() {}

    public void execute()
    {
        if (!reverse)
        {
            m_subsystem.run(0.6);
        }
        else
        {
            m_subsystem.run(-0.6);
        }
    }

    public void end(boolean interrupted)
    {
        m_subsystem.run(0);
    }

    public boolean isFinished() {return false;}
}
*/