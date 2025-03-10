package frc.robot.controllers;

import java.util.Optional;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
//import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import entechlib.util.EntechUtils;
import frc.robot.Constants;

public class SparkMaxPositionController implements Sendable, PositionController{
    
    public enum HomingState {
		FINDING_LIMIT, HOMED, UNINITIALIZED
	}

	protected SparkMax spark;
	private HomingState axisState = HomingState.UNINITIALIZED;

	private PositionControllerConfig config;
	private RelativeEncoder encoder;
	private SparkLimitSwitch lowerLimit;
	private Optional<Double> requestedPosition = Optional.empty();
	private SparkLimitSwitch upperLimit;
	private boolean speedMode = false;

	public SparkMaxPositionController(SparkMax spark, PositionControllerConfig config,
			SparkLimitSwitch lowerLimit,
			SparkLimitSwitch upperLimit, RelativeEncoder encoder) {
		this.spark = spark;

		if (config.isInverted()) {
			this.lowerLimit = upperLimit;
			this.upperLimit = lowerLimit;
		} else {
			this.lowerLimit = lowerLimit;
			this.upperLimit = upperLimit;
		}

		this.encoder = encoder;
		this.config = config;
		clearRequestedPosition();
	}

	public void clearRequestedPosition() {
		this.requestedPosition = Optional.empty();
		setMotorSpeedInternal(0.0);
	}

	@Override
	public double getActualPosition() {
		return getEncoderValue();
	}

	public PositionControllerConfig getConfig() {
		return this.config;
	}

	@Override
	public HomingState getMotionState() {
		return axisState;
	}

	public double getMotorOutput() {
		return spark.getAppliedOutput();
	}

	@Override
	public double getRequestedPosition() {
		if (requestedPosition.isPresent()) {
			return requestedPosition.get();
		} else {
			return Constants.INDICATOR_VALUES.POSITION_NOT_SET;
		}
	}

	public String getStatusString() {
		if (axisState == HomingState.HOMED) {
			if (requestedPosition.isPresent()) {
				if (this.isAtRequestedPosition()) {
					return "ARRIVED";
				} else {
					return "MOVING";
				}
			} else {
				return "HOMED";
			}
		} else {
			return this.axisState + "";
		}
	}

	public void home() {
		startHoming();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("PositionController:" + config.getName());
		builder.addStringProperty("Status:", this::getStatusString, null);
		builder.addDoubleProperty("RequestedPos", this::getRequestedPosition, null);
		builder.addDoubleProperty("ActualPos", this::getActualPosition, null);
		// builder.addBooleanProperty("InPosition", this::isAtRequestedPosition, null);
		builder.addBooleanProperty("UpperLimit", this::isAtUpperLimit, null);
		builder.addBooleanProperty("LowerLimit", this::isAtLowerLimit, null);
		// builder.addDoubleProperty("MotorOut", () -> { return
		// spark.getAppliedOutput();}, null);
		builder.addDoubleProperty("MotorCurrent", () -> {
			return spark.getOutputCurrent();
		}, null);

	}

	@Override
	public boolean isAtLowerLimit() {
		return lowerLimit.isPressed();
	}

	// after this call, another request to resetPosition is required to do homing
	// again
	public void forgetHome() {
		axisState = HomingState.UNINITIALIZED;
		this.clearRequestedPosition();
	}

	@Override
	public boolean isAtRequestedPosition() {
		if (isHomed()) {
			if (requestedPosition.isPresent()) {
				return Math.abs(getActualPosition() - requestedPosition.get()) < config.getPositionTolerance();
			}
		}
		return false;
	}

	@Override
	public boolean isAtUpperLimit() {
		return upperLimit.isPressed();
	}

	@Override
	public boolean isHomed() {
		return axisState == HomingState.HOMED;
	}

	@Override
	public void requestPosition(double requestedPosition) {
		this.requestedPosition = Optional
				.of(EntechUtils.capDoubleValue(requestedPosition, config.getMinPosition(), config.getMaxPosition()));
		speedMode = false;
		if (axisState == HomingState.UNINITIALIZED) {
			startHoming();
		}
	}

	public void setPositionMode() {
		speedMode = false;
	}

	public void setMotorSpeed(double input) {
		setMotorSpeedInternal(input);
	}

	public void setMotorSpeedInternal(double input) {
		if (config.isHomeClosedLoop()) {
			spark.getClosedLoopController().setReference(correctDirection(input), SparkMax.ControlType.kVelocity);
		} else {
			spark.set(correctDirection(input));
		}
		speedMode = true;
	}

	public void stop() {
		spark.stopMotor();
	}

	@Override
	public void update() {
		switch (axisState) {
			case UNINITIALIZED:
				if (isAtLowerLimit()) {
					arrivedHome();
				}
				break;
			case FINDING_LIMIT:
				speedMode = true;
				if (isAtLowerLimit()) {
					arrivedHome();
				} else {
					setMotorSpeedInternal(-config.getHomingSpeedPercent());
				}
				break;
			case HOMED:
				updateRequestedPosition();
				if (isAtLowerLimit()) {
					setEncoder(config.getMinPosition());
				}
		}

	}

	private void arrivedHome() {
		setMotorSpeedInternal(0);
		setEncoder(config.getMinPosition());
		axisState = HomingState.HOMED;
		speedMode = false;
	}

	private double correctDirection(double input) {
		if (config.isInverted()) {
			return -input;
		} else {
			return input;
		}
	}

	private double getEncoderValue() {
		return correctDirection(encoder.getPosition());
	}

	private void setEncoder(double value) {
		encoder.setPosition(correctDirection(value));
	}

	private void startHoming() {
		// if we are already on the limit switch, we'll get homed in the next update
		// loop
		setMotorSpeedInternal(-config.getHomingSpeedPercent());
		axisState = HomingState.FINDING_LIMIT;
	}

	private void updateRequestedPosition() {
		if (!speedMode) {
			if (requestedPosition.isPresent()) {
				spark.getClosedLoopController().setReference(correctDirection(requestedPosition.get()),
						SparkMax.ControlType.kPosition);
			}
		}
	}

}
