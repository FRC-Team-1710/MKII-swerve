package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.robot.Utilities;

/**
 * Command class for sending controller values to
 * {@link com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem#drive(Translation2d, double, boolean)}
 */
public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
        // Filter out values less than 0.1
        forward = Utilities.deadband(forward, 0.1);
        // Square the forward stick but keep the sign
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
        // Filter out values less than 0.1
        strafe = Utilities.deadband(strafe, 0.1);
        // Square the strafe stick but keep the sign
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.getOi().getPrimaryJoystick().getRawAxis(4);
        // Filter out values less than 0.15
        rotation = Utilities.deadband(rotation, 0.15);
        // Square the rotation stick but keep the sign
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        // Send values to drive method in DrivetrainSubsystem
        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}