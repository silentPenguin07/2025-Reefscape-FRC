/*
package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;;

public class CoralIntakeSubsystem extends SubsystemBase {

    private SparkMax m_intake;

    public CoralIntakeSubsystem()
    {
        @SuppressWarnings({ "resource", "unused" })
        SparkMax m_intake = new SparkMax(Constants.ElevatorConstants.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
    }

    public void intake(boolean reverse)
    {
        if (!reverse)
        {
            m_intake.set(0.4);
        }
        else{
            m_intake.set(-0.4);
        }
    }

    public void brake()
    {
        m_intake.set(0);
    }

}*/