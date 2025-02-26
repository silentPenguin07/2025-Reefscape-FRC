package frc.robot.robot_swerve;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.sensors.ThriftyEncoder;

public class SwerveModule {
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final ThriftyEncoder m_turningAbsoluteEncoder;

    private final SparkClosedLoopController m_drivingPidController;
    private final SparkClosedLoopController m_turningPidController;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller.
     */
    public SwerveModule(int drivingCANId, int turningCANId, int turningAnalogPort, boolean drivingInverted) {
        m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        System.out.println(m_drivingSparkMax.getDeviceId());
        System.out.println(m_turningSparkMax.getDeviceId());

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getEncoder();
        m_turningAbsoluteEncoder = new ThriftyEncoder(turningAnalogPort);

        m_drivingPidController = m_drivingSparkMax.getClosedLoopController(); // TODO: SET REFERENCE
        m_turningPidController = m_turningSparkMax.getClosedLoopController(); // TODO: SET REFERENCE

        // object for configuring driving motor
        SparkMaxConfig drivingConfig = new SparkMaxConfig();

        drivingConfig
                .inverted(drivingInverted)
                .idleMode(SwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
        drivingConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION)
                .velocityConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);
        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: may have to be kAnalogueEncoder???
                .p(SwerveModuleConstants.DRIVING_P) 
                .i(SwerveModuleConstants.DRIVING_I) 
                .d(SwerveModuleConstants.DRIVING_D)
                .velocityFF(SwerveModuleConstants.DRIVING_FF)
                .outputRange(SwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED, SwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);
        
        // link sparkmax and configuration
        m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_drivingSparkMax.clearFaults();
        
        // object for configuring turning motor
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig
                .inverted(true) // always inverted; try not inverting it for a different driving experience
                .idleMode(SwerveModuleConstants.TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
        turningConfig.encoder
                .positionConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION)
                .velocityConversionFactor(SwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM);
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: maybe primary sensor??
                .pid(SwerveModuleConstants.TURNING_P, SwerveModuleConstants.TURNING_I, SwerveModuleConstants.TURNING_D)
                .positionWrappingEnabled(true) // PID wrapparound so wheel doesn't have to spin more than 180 degrees
                .positionWrappingMinInput(SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS)
                .positionWrappingMaxInput(SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS)
                .velocityFF(SwerveModuleConstants.TURNING_FF)
                .outputRange(SwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED,
                SwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED);

        m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_turningSparkMax.clearFaults();

        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        // Optimize the reference state to avoid spinning further than 90 degrees.
        /* DEPRECIATED
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition()));
        */ 
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001 // less than 1 mm per sec
                && Math.abs(correctedDesiredState.angle.getRadians() - m_turningEncoder.getPosition()) < 0.1) // 10% of
                                                                                                              // a
                                                                                                              // radian
        {
            m_drivingSparkMax.set(0); // no point in doing anything
            m_turningSparkMax.set(0);
        } else {
            // Command driving and turning SPARKS MAX towards their respective setpoints.
            m_drivingSparkMax.getClosedLoopController().setReference(correctedDesiredState.speedMetersPerSecond,
                    SparkMax.ControlType.kVelocity);
            m_turningSparkMax.getClosedLoopController().setReference(correctedDesiredState.angle.getRadians(),
                    SparkMax.ControlType.kPosition);
        }

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule relative encoders. */
    public void resetEncoders() {

        m_drivingEncoder.setPosition(0); // arbitrarily set driving encoder to zero

        // temp
        // m_turningAbsoluteEncoder.resetVirtualPosition();
        // the reading and setting of the calibrated absolute turning encoder values is
        // done in the Drivetrain's constructor

        m_turningSparkMax.set(0); // no moving during reset of relative turning encoder

        m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getVirtualPosition()); // set relative position based on
                                                                                     // virtual absolute position
    }

    /**
     * Calibrates the virtual position (i.e. sets position offset) of the absolute
     * encoder.
     */
    public void calibrateVirtualPosition(double angle) {
        m_turningAbsoluteEncoder.setPositionOffset(angle);
    }

    public RelativeEncoder getDrivingEncoder() {
        return m_drivingEncoder;
    }

    public RelativeEncoder getTurningEncoder() {
        return m_turningEncoder;
    }

    public ThriftyEncoder getTurningAbsoluteEncoder() {
        return m_turningAbsoluteEncoder;
    }

    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

}
