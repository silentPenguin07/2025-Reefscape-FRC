/*
package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ClimbSubsystem extends SubsystemBase {

    private SparkMax m_climb_motor;

    public ClimbSubsystem()
    {
        m_climb_motor = new SparkMax(13, MotorType.kBrushless);

        SparkMaxConfig climbConfig = new SparkMaxConfig();

        climbConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        m_climb_motor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climb_motor.clearFaults();

        m_climb_motor.getEncoder().setPosition(0.0);
    }

    public void run(double speed)
    {
        m_climb_motor.set(speed);
    }

    public void stop()
    {
        m_climb_motor.set(0);
    }

    public void periodic()
    {

    }

}
*/