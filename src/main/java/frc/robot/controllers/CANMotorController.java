/*
package frc.robot.controllers;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CANMotorController implements MotorController{
    public static enum MotorControllerType {
        SMAX_BRUSHED,
        SMAX_BRUSHLESS
    }

    private final MotorControllerType type;
    private SparkMax motorControllerSMAX;
    
    // constructor initializes the motor controller
    public CANMotorController(int CANID, MotorControllerType Type) {
        type = Type;

        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX = new SparkMax(CANID, MotorType.kBrushed);
            case SMAX_BRUSHLESS:
                motorControllerSMAX = new SparkMax(CANID, MotorType.kBrushless);
        }
    }

    @Override
    public void set(double speed) {
        motorControllerSMAX.set(speed);
    }

    @Override
    public double get() {
        return motorControllerSMAX.get();
    }

    @Override
    public void disable() {
        motorControllerSMAX.disable();
    }

    @Override
    public void stopMotor() {
        motorControllerSMAX.stopMotor();
    }
    
    @Override
    public void setInverted(boolean inverted) {
        motorControllerSMAX.setInverted(inverted); // depreciated; use something else in the future
    }

    @Override
    public boolean getInverted() {
        return motorControllerSMAX.getInverted(); // depreciated; use something else in the future
    }

    public void feed() {
        motorControllerSMAX.set(get());
    }
}
    */
