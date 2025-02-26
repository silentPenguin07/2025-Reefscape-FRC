
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


public class CoralElevatorSubsystem extends SubsystemBase {
    
    // Motor controllers
    private SparkMax m_elevator_leader; // NEO for moving arm upwards
    private SparkMax m_elevator_follower; // follower NEO

    private double gravityControl; // TODO: check this calculation

    // limiters TODO: test these
    public double arm_max = Constants.ElevatorConstants.ELEVATOR_ARM_MAX;
    public double arm_min = Constants.ElevatorConstants.ELEVATOR_ARM_MIN;

    public CoralElevatorSubsystem()
    {
        m_elevator_leader = new SparkMax(Constants.ElevatorConstants.ELEVATOR_LEAD_MOTOR_ID, MotorType.kBrushless);
        m_elevator_follower = new SparkMax(Constants.ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        // configure the sparkmaxes
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        boolean invert = false;

        leaderConfig
            .inverted(invert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.ElevatorConstants.kP);
        m_elevator_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevator_leader.clearFaults();

        followerConfig
            //.inverted(!invert)
            .follow(m_elevator_leader, invert) // follower in opposing direction but follows same Closed Loop
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        m_elevator_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevator_follower.clearFaults();

        m_elevator_leader.getEncoder().setPosition(0.0);
    }


    public double getElevatorPosition()
    {
        
        return m_elevator_leader.getAbsoluteEncoder().getPosition(); // maybe just getEncoder()???
    }

    public void setElevatorSpeed(double speed)
    {
        m_elevator_leader.set(speed);
    }

    public void stopElevator()
    {
        m_elevator_leader.set(0);
    }

    public double getGravityControl()
    {
        return gravityControl;
    }

    public void periodic()
    {
        gravityControl = Math.sin((getElevatorPosition() / 70 * 2 * Math.PI) + Math.PI / 2) * 
            Constants.ElevatorConstants.ELEVATOR_GRAVITY_CONST;

        SmartDashboard.putNumber("Elevator Angle", getElevatorPosition());
        SmartDashboard.putNumber("Gravity Control", gravityControl);
    }

}
