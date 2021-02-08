package com.swervedrivespecialties.exampleswerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.commands.autonomousDrive;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
        private static OI oi;
        private Command autonomousCommand;
        private static DrivetrainSubsystem drivetrain;
        private CANSparkMax backLeftAngle = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRightAngle = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeftAngle = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontRightAngle = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                        CANSparkMaxLowLevel.MotorType.kBrushless);

        public static OI getOi() {
                return oi;
        }

        @Override
        public void robotInit() {
                oi = new OI();
                drivetrain = DrivetrainSubsystem.getInstance();
                backLeftAngle.setSmartCurrentLimit(25);
                backRightAngle.setSmartCurrentLimit(25);
                backLeftDrive.setSmartCurrentLimit(25);
                backRightDrive.setSmartCurrentLimit(25);
                frontLeftAngle.setSmartCurrentLimit(25);
                frontLeftDrive.setSmartCurrentLimit(25);
                frontRightAngle.setSmartCurrentLimit(25);
                frontRightDrive.setSmartCurrentLimit(25);
        }

        @Override
        public void robotPeriodic() {
                Scheduler.getInstance().run();
        }

        @Override
        public void autonomousInit() {
                autonomousCommand = new autonomousDrive();
                if (autonomousCommand != null)
                        autonomousCommand.start();
        }

        @Override
        public void autonomousPeriodic() {
        }
}