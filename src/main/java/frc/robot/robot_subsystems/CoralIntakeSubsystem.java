package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;;

public class CoralIntakeSubsystem extends SubsystemBase {

    private SparkMax m_leaderSparkMax;
    private SparkMax m_followerSparkMax;
    private boolean inverted = false;

    public CoralIntakeSubsystem() {
        m_leaderSparkMax = new SparkMax(Constants.ElevatorConstants.CORAL_LEAD_INTAKE_MOTOR_ID, MotorType.kBrushless);
        m_followerSparkMax = new SparkMax(Constants.ElevatorConstants.CORAL_FOLLOWER_INTAKE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        leaderConfig // configuration settings
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        // link sparkmax and configuration
        m_leaderSparkMax.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leaderSparkMax.clearFaults();

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig // configuration settings
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .follow(m_leaderSparkMax, true);
        
        // link sparkmax and configuration
        m_followerSparkMax.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_followerSparkMax.clearFaults();
    }

    public void intake(boolean reverse)
    {
        if (!reverse)
        {
            m_leaderSparkMax.set(0.7);
        }
        else{
            m_leaderSparkMax.set(-0.7);
        }
    }

    public void brake()
    {
        m_leaderSparkMax.set(0);
    }

}