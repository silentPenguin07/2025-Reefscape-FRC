package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import entechlib.commands.EntechCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.OI.UserPolicy;
import frc.robot.robot_subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private static final double MAX_SPEED_PERCENT = 1;

    private final DriveSubsystem drive;
    private final XboxController joystick;

    public DriveCommand(DriveSubsystem drive, XboxController joystick) {
        //super(drive);
        this.drive = drive;
        this.joystick = joystick;
        addRequirements(drive); // ALWAYS add subsystems as a requirement
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true, true);
    }

    @Override
    public void execute() {
        double xRaw = joystick.getLeftX();
        double yRaw = joystick.getLeftY();
        
        double rotRaw = -joystick.getRightX(); //Change to getRightY if the robot is up and down instead of left and right

        double xConstrained = MathUtil.applyDeadband(MathUtil.clamp(xRaw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT),
                Constants.Ports.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);
        double yConstrained = MathUtil.applyDeadband(MathUtil.clamp(yRaw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT),
                Constants.Ports.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);
        double rotConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(rotRaw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT),
                Constants.Ports.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);

        double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
        double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
        double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

        if (UserPolicy.xLocked) {
            drive.setX();
            return;
        }

        if (UserPolicy.twistable) {
            drive.drive(-ySquared, -xSquared, -rotSquared, true, true);
        } else {
            drive.drive(-ySquared, -xSquared, 0, true, true);
        }
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
