/*
package frc.robot.robot_subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private SparkMax m_leader, m_follower;
    private SparkMaxConfig leader_config, follower_config;

    public AlgaeIntakeSubsystem()
    {
        // create motors
        m_leader = new SparkMax(Constants.ElevatorConstants.ALGAE_INTAKE_LEADER_MOTOR_ID, MotorType.kBrushless);
        m_follower = new SparkMax(Constants.ElevatorConstants.ALGAE_INTAKE_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        
        leader_config = new SparkMaxConfig();
        leader_config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        m_leader.configure(leader_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        follower_config = new SparkMaxConfig();
        follower_config
            .idleMode(IdleMode.kBrake)
            .follow(m_leader, true);
            m_follower.configure(follower_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void revAlgaeIntake(boolean reverse)
    {
        if (reverse) {m_leader.set(0-0.2);}
        else {m_leader.set(0+0.2);}
    }

    public void brake() {m_leader.set(0);}
}
*/